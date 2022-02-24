package frc.swervelib;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;

public class SwerveDrivetrainModel {

    QuadSwerveSim swerveDt;
    ArrayList<SwerveModule> realModules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);
    ArrayList<SwerveModuleSim> modules = new ArrayList<SwerveModuleSim>(QuadSwerveSim.NUM_MODULES);

    ArrayList<SteerController> steerMotorControllers = new ArrayList<SteerController>(QuadSwerveSim.NUM_MODULES);
    ArrayList<DriveController> driveMotorControllers = new ArrayList<DriveController>(QuadSwerveSim.NUM_MODULES);
    ArrayList<AbsoluteEncoder> steerEncoders = new ArrayList<AbsoluteEncoder>(QuadSwerveSim.NUM_MODULES);

    Gyroscope gyro;

    Pose2d endPose;
    PoseTelemetry dtPoseView;
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(SwerveConstants.KINEMATICS, getGyroscopeRotation());

    SwerveDrivePoseEstimator m_poseEstimator;
    Pose2d curEstPose = new Pose2d(SwerveConstants.DFLT_START_POSE.getTranslation(), SwerveConstants.DFLT_START_POSE.getRotation());
    Pose2d fieldPose = new Pose2d(); // Field-referenced orign
    boolean pointedDownfield = false;
    double curSpeed = 0;
    SwerveModuleState[] states;
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            SwerveConstants.THETACONTROLLERkP, 0, 0, SwerveConstants.THETACONTROLLERCONSTRAINTS);

    HolonomicDriveController m_holo;
    
    private static final SendableChooser<String> orientationChooser = new SendableChooser<>();

    public SwerveDrivetrainModel(ArrayList<SwerveModule> realModules, Gyroscope gyro){
        this.gyro = gyro;
        this.realModules = realModules;

        if (RobotBase.isSimulation()) {
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(0)));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(1)));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(2)));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(3)));
        }
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        endPose = SwerveConstants.DFLT_START_POSE;

        swerveDt = new QuadSwerveSim(SwerveConstants.TRACKWIDTH_METERS,
                                    SwerveConstants.TRACKLENGTH_METERS,
                                    SwerveConstants.MASS_kg,
                                    SwerveConstants.MOI_KGM2,
                                    modules);

        // Trustworthiness of the internal model of how motors should be moving
        // Measured in expected standard deviation (meters of position and degrees of
        // rotation)
        var stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

        // Trustworthiness of gyro in radians of standard deviation.
        var localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.1));

        // Trustworthiness of the vision system
        // Measured in expected standard deviation (meters of position and degrees of
        // rotation)
        var visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

        m_poseEstimator = new SwerveDrivePoseEstimator(getGyroscopeRotation(), SwerveConstants.DFLT_START_POSE,
                SwerveConstants.KINEMATICS, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs,
                SimConstants.CTRLS_SAMPLE_RATE_SEC);

        setKnownPose(SwerveConstants.DFLT_START_POSE);

        dtPoseView = new PoseTelemetry(swerveDt, m_poseEstimator);

        // Control Orientation Chooser
        orientationChooser.setDefaultOption("Field Oriented", "Field Oriented");
        orientationChooser.addOption("Robot Oriented", "Robot Oriented");
        SmartDashboard.putData("Orientation Chooser", orientationChooser);

        m_holo = new HolonomicDriveController(SwerveConstants.XPIDCONTROLLER, SwerveConstants.YPIDCONTROLLER, thetaController);
    }

    /**
     * Handles discontinuous jumps in robot pose. Used at the start of
     * autonomous, if the user manually drags the robot across the field in the
     * Field2d widget, or something similar to that.
     * @param pose The new pose the robot is "jumping" to.
     */
    public void modelReset(Pose2d pose){
        swerveDt.modelReset(pose);
        m_odometry.resetPosition(pose, getGyroscopeRotation());
    }

    /**
     * Advance the simulation forward by one step
     * @param isDisabled Boolean that indicates if the robot is in the disabled mode
     * @param batteryVoltage Amount of voltage available to the drivetrain at the current step.
     */
    public void update(boolean isDisabled, double batteryVoltage){
        // Calculate and update input voltages to each motor.
        if(isDisabled){
            for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
                modules.get(idx).setInputVoltages(0.0, 0.0);
            }
        } else {
            for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
                double steerVolts = realModules.get(idx).getSteerController().getOutputVoltage();
                double wheelVolts = realModules.get(idx).getDriveController().getOutputVoltage();
                modules.get(idx).setInputVoltages(wheelVolts, steerVolts);
            }
        }

        //Update the main drivetrain plant model
        swerveDt.update(SimConstants.SIM_SAMPLE_RATE_SEC);

        // Update each encoder
        for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
            double azmthShaftPos = modules.get(idx).getAzimuthEncoderPositionRev();
            double steerMotorPos = modules.get(idx).getAzimuthMotorPositionRev();
            double wheelPos = modules.get(idx).getWheelEncoderPositionRev();

            double azmthShaftVel = modules.get(idx).getAzimuthEncoderVelocityRPM();
            double steerVelocity = modules.get(idx).getAzimuthMotorVelocityRPM();
            double wheelVelocity = modules.get(idx).getWheelEncoderVelocityRPM();

            realModules.get(idx).getAbsoluteEncoder().setAbsoluteEncoder(azmthShaftPos, azmthShaftVel);
            realModules.get(idx).getSteerController().setSteerEncoder(steerMotorPos, steerVelocity);
            realModules.get(idx).getDriveController().setDriveEncoder(wheelPos, wheelVelocity);
        }

        // Update associated devices based on drivetrain motion
        gyro.setAngle(swerveDt.getCurPose().getRotation().getDegrees());

        // Based on gyro and measured module speeds and positions, estimate where our
        // robot should have moved to.
        Pose2d prevEstPose = curEstPose;
        if (states != null) {
            curEstPose = m_poseEstimator.update(getGyroscopeRotation(), states[0], states[1], states[2], states[3]);
        
            // Calculate a "speedometer" velocity in ft/sec
            Transform2d chngPose = new Transform2d(prevEstPose, curEstPose);
            curSpeed = Units.metersToFeet(chngPose.getTranslation().getNorm()) / SimConstants.CTRLS_SAMPLE_RATE_SEC;

            updateDownfieldFlag();
        }
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        states = desiredStates;
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param chassisSpeeds The desired SwerveModule states.
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        states = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    }

    public void setModuleStates(SwerveInput input) {
        switch (orientationChooser.getSelected()) {
            case "Field Oriented":
                states = SwerveConstants.KINEMATICS.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                input.m_translationX * SwerveConstants.MAX_FWD_REV_SPEED_MPS,
                                input.m_translationY * SwerveConstants.MAX_STRAFE_SPEED_MPS,
                                input.m_rotation * SwerveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC,
                                getGyroscopeRotation()
                        )    
                );
                break;
            case "Robot Oriented":
                states = SwerveConstants.KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds(
                                input.m_translationX * SwerveConstants.MAX_FWD_REV_SPEED_MPS,
                                input.m_translationY * SwerveConstants.MAX_STRAFE_SPEED_MPS,
                                input.m_rotation * SwerveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC
                        )  
                );
                break;
        }
    }

    public SwerveModuleState[] getSwerveModuleStates() {
      return states;
    }

    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    public Pose2d getEstPose() {
        return curEstPose;
    }

    public void setKnownPose(Pose2d in) {
        m_odometry.resetPosition(in, getGyroscopeRotation());
        resetWheelEncoders();
        m_poseEstimator.resetPosition(in, getGyroscopeRotation());
        zeroGyroscope();
        updateDownfieldFlag();
        curEstPose = in;
    }

    public void updateDownfieldFlag() {
      double curRotDeg = curEstPose.getRotation().getDegrees();
      pointedDownfield = (curRotDeg > -90 && curRotDeg < 90);
    }

    public void zeroGyroscope() {
        gyro.zeroGyroscope();
    }

    public Rotation2d getGyroscopeRotation() {
        return gyro.getGyroHeading();
    }

    public Boolean getGyroReady() {
        return gyro.getGyroReady();
    }

    public void updateTelemetry(){
        dtPoseView.update(Timer.getFPGATimestamp()*1000);
    }

    public Field2d getField() {
        return dtPoseView.getField();
    }

    public void resetWheelEncoders() {
      for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
        realModules.get(idx).resetWheelEncoder();
      }
    }

    public Command createCommandForTrajectory(PathPlannerTrajectory trajectory, SwerveSubsystem m_drive) {
        PPSwerveControllerCommand swerveControllerCommand =
            new PPSwerveControllerCommand(
                trajectory,
                () -> getPose(), // Functional interface to feed supplier
                SwerveConstants.KINEMATICS,

                // Position controllers
                SwerveConstants.XPIDCONTROLLER,
                SwerveConstants.YPIDCONTROLLER,
                thetaController,
                commandStates -> this.states = commandStates,
                m_drive);
        return swerveControllerCommand.andThen(() -> setModuleStates(new SwerveInput(0,0,0)));
    }

    public ArrayList<SwerveModule> getRealModules() {
        return realModules;
    }

    public ArrayList<SwerveModuleSim> getModules() {
        return modules;
    }

    public void goToPose(Pose2d desiredPose, double angle) {
        setModuleStates(m_holo.calculate(getPose(), desiredPose, 1, Rotation2d.fromDegrees(angle)));
    }

    public void updateOdometry() {
        states[0].speedMetersPerSecond = Math.abs(realModules.get(0).getDriveVelocity());
        states[1].speedMetersPerSecond = Math.abs(realModules.get(1).getDriveVelocity());
        states[2].speedMetersPerSecond = Math.abs(realModules.get(2).getDriveVelocity());
        states[3].speedMetersPerSecond = Math.abs(realModules.get(3).getDriveVelocity());
        m_odometry.update(getGyroscopeRotation(), states);
    }
}