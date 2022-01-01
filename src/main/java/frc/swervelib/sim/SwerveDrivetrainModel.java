package frc.swervelib.sim;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.swervelib.SwerveControllerCommandPP;
import frc.swervelib.SwerveSubsystem;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;
import frc.swervelib.AbsoluteEncoder;
import frc.swervelib.DriveController;
import frc.swervelib.Gyroscope;
import frc.swervelib.Mk4SwerveModuleHelper;
import frc.swervelib.PoseTelemetry;
import frc.swervelib.SimConstants;
import frc.swervelib.SteerController;
import frc.swervelib.SwerveModule;
import frc.swervelib.SwerveConstants;

public class SwerveDrivetrainModel {

    QuadSwerveSim swerveDt;
    ArrayList<SwerveModule> realModules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);
    ArrayList<SwerveModuleSim> modules = new ArrayList<SwerveModuleSim>(QuadSwerveSim.NUM_MODULES);

    ArrayList<SteerController> steerMotorControllers = new ArrayList<SteerController>(QuadSwerveSim.NUM_MODULES);
    ArrayList<DriveController> driveMotorControllers = new ArrayList<DriveController>(QuadSwerveSim.NUM_MODULES);
    ArrayList<AbsoluteEncoder> steerEncoders = new ArrayList<AbsoluteEncoder>(QuadSwerveSim.NUM_MODULES);

    Gyroscope gyro;

    Field2d field;
    Pose2d endPose;
    PoseTelemetry dtPoseView;

    SwerveDrivePoseEstimator m_poseEstimator;
    Pose2d curEstPose = new Pose2d(SwerveConstants.DFLT_START_POSE.getTranslation(), SwerveConstants.DFLT_START_POSE.getRotation());
    Pose2d fieldPose = new Pose2d(); // Field-referenced orign
    boolean pointedDownfield = false;
    double curSpeed = 0;
    SwerveModuleState[] states;
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            SwerveConstants.THETACONTROLLERkP, 0, 0, SwerveConstants.THETACONTROLLERCONSTRAINTS);

    public SwerveDrivetrainModel(ArrayList<SwerveModule> realModules, Gyroscope gyro){
        this.gyro = gyro;
        this.realModules = realModules;

        if (RobotBase.isSimulation()) {
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(0), "FL"));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(1), "FR"));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(2), "BL"));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(3), "BR"));
        }
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        field = PoseTelemetry.field;
        field.setRobotPose(SwerveConstants.DFLT_START_POSE);
        endPose = SwerveConstants.DFLT_START_POSE;
        dtPoseView = new PoseTelemetry();

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
    }

    /**
     * Handles discontinuous jumps in robot pose. Used at the start of
     * autonomous, if the user manually drags the robot across the field in the
     * Field2d widget, or something similar to that.
     * @param pose
     */
    public void modelReset(Pose2d pose){
        field.setRobotPose(pose);
        swerveDt.modelReset(pose);
        resetPose(pose);
    }

    /**
     * Advance the simulation forward by one step
     * @param isDisabled
     * @param batteryVoltage
     */
    public void update(boolean isDisabled, double batteryVoltage){
 
        // Check if the user moved the robot with the Field2D
        // widget, and reset the model if so.
        Pose2d startPose = field.getRobotPose();
        Transform2d deltaPose = startPose.minus(endPose);
        if(deltaPose.getRotation().getDegrees() > 0.01 || deltaPose.getTranslation().getNorm() > 0.01){
            modelReset(startPose);
        }

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
        endPose = swerveDt.getCurPose();

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
        field.setRobotPose(endPose);
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
      this.states = desiredStates;
    }

    public SwerveModuleState[] getSwerveModuleStates() {
      return states;
    }

    public Pose2d getCurActPose(){
        return field.getRobotObject().getPose();
    }

    public void resetPose(Pose2d pose){
        modelReset(pose);
    }

    public Pose2d getEstPose() {
        return curEstPose;
    }

    public void setKnownPose(Pose2d in) {
        resetWheelEncoders();
        // No need to reset gyro, pose estimator does that.
        m_poseEstimator.resetPosition(in, getGyroscopeRotation());
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

    public void updateTelemetry(){
        if(RobotBase.isSimulation()){
            dtPoseView.setActualPose(getCurActPose());
        }
        dtPoseView.setEstimatedPose(getEstPose());
        //dtPoseView.setDesiredPose(getCurDesiredPose());

        dtPoseView.update(Timer.getFPGATimestamp()*1000);
    }

    public void resetWheelEncoders() {
      for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
        realModules.get(idx).resetWheelEncoder();
      }
    }

    public Command createCommandForTrajectory(PathPlannerTrajectory trajectory, SwerveSubsystem m_drive) {
        SwerveControllerCommandPP swerveControllerCommand =
            new SwerveControllerCommandPP(
                trajectory,
                () -> getCurActPose(), // Functional interface to feed supplier
                SwerveConstants.KINEMATICS,

                // Position controllers
                new PIDController(SwerveConstants.TRAJECTORYXkP, 0, 0),
                new PIDController(SwerveConstants.TRAJECTORYYkP, 0, 0),
                thetaController,
                commandStates -> setModuleStates(commandStates),
                m_drive);
        return swerveControllerCommand;
    }
}