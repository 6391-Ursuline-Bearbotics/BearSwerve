package frc.wpiClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveModuleSim {

    private SimpleMotorWithMassModel steerMotor;
    private MotorGearboxWheelSim driveMotor;

    private final double azimuthEncGearRatio;    //Motor-to-azimuth-encoder reduction
    private final double wheelEncGearRatio;      //Motor-to-wheel-encoder reduction
    private final double treadStaticFricForce;
    private final double treadKineticFricForce;
    private final double wheelGearboxLossFactor = 0.01; //TODO - make the "how much grease" factor configurable?

    Pose2d prevModulePose = null;
    Pose2d curModulePose  = null;
    double curLinearSpeed_mps = 0; //Positive = in curAngle_deg, Negative = opposite of curAngle_deg
    Rotation2d cursteerAngle = Rotation2d.fromDegrees(0); //0 = toward front, 90 = toward left, 180 = toward back, 270 = toward right

    double crossTreadFricForceMag = 0;
    double crossTreadVelMag = 0;
    double crossTreadForceMag = 0;

    double wheelVoltage;
    double steerVoltage;

    private ShuffleboardTab tab = Shuffleboard.getTab("SimSwerve");
    private NetworkTableEntry azmthVoltageEntry;
    private NetworkTableEntry wheelVoltageEntry;
    private NetworkTableEntry azmthSpeedEntry;
    private NetworkTableEntry azmthPosEntry;
    private NetworkTableEntry wheelSpeedEntry;

    public SwerveModuleSim(
        DCMotor azimuthMotor,
        DCMotor driveMotor, 
        double wheelRadius_m,
        double azimuthGearRatio,      // Motor rotations per one azimuth module rotation. Should be greater than zero
        double wheelGearRatio,        // Motor rotations per one wheel rotation. Should be greater than zero
        double azimuthEncGearRatio,   // Encoder rotations per one azimuth module rotation. Should be 1.0 if you have a good swerve module.
        double wheelEncGearRatio,     // Encoder rotations per one wheel rotation.
        double treadStaticCoefFric,
        double treadKineticCoefFric,
        double moduleNormalForce,
        double azimuthEffectiveMOI,
        String namePrefix
    ){
        this.steerMotor = new SimpleMotorWithMassModel(azimuthMotor, azimuthGearRatio, azimuthEffectiveMOI);
        this.driveMotor = new MotorGearboxWheelSim(driveMotor, wheelGearRatio, wheelRadius_m, wheelGearboxLossFactor);
     
        this.azimuthEncGearRatio   = azimuthEncGearRatio; 
        this.wheelEncGearRatio     = wheelEncGearRatio;   
        this.treadStaticFricForce  = treadStaticCoefFric*moduleNormalForce;
        this.treadKineticFricForce = treadKineticCoefFric*moduleNormalForce; 

        this.azmthVoltageEntry = tab.add(namePrefix + "Azmth Voltage V", 0).getEntry();
        this.wheelVoltageEntry = tab.add(namePrefix + "Wheel Voltage V", 0).getEntry();
        this.azmthSpeedEntry = tab.add(namePrefix + "Azmth Speed RPM", 0).getEntry();
        this.azmthPosEntry = tab.add(namePrefix + "Azmth Pos Deg", 0).getEntry();
        this.wheelSpeedEntry = tab.add(namePrefix + "Wheel Speed RPM", 0).getEntry(); 
    }

    public void setInputVoltages(double wheelVoltage, double steerVoltage){
        this.wheelVoltage = wheelVoltage;
        this.steerVoltage = steerVoltage;
    }

    public double getAzimuthMotorPositionRev(){
        return steerMotor.getMotorPosition_Rev();
    }

    public double getAzimuthMotorVelocityRPM(){
        return steerMotor.getMotorSpeed_RPM();
    }

    public double getAzimuthEncoderPositionRev(){
        return steerMotor.getMechanismPosition_Rev() * azimuthEncGearRatio;
    }

    public double getAzimuthEncoderVelocityRPM(){
        return steerMotor.getMechanismSpeed_RPM() * azimuthEncGearRatio;
    }

    public double getWheelEncoderPositionRev(){
        return driveMotor.getPosition_Rev() * wheelEncGearRatio;
    }

    public double getWheelEncoderVelocityRPM(){
        return steerMotor.getMechanismSpeed_RPM() * wheelEncGearRatio;
    }

    void reset(Pose2d initModulePose){
        prevModulePose = curModulePose = initModulePose;
        curLinearSpeed_mps = 0;
        cursteerAngle = Rotation2d.fromDegrees(0);
    }

    void update(double dtSeconds){

        Vector2d azimuthUnitVec = new Vector2d(1,0);
        azimuthUnitVec.rotate(cursteerAngle.getDegrees());

        // Assume the wheel does not lose traction along its wheel direction (on-tread)
        double velocityAlongAzimuth = getModuleRelativeTranslationVelocity(dtSeconds).dot(azimuthUnitVec);

        driveMotor.update(velocityAlongAzimuth, wheelVoltage, dtSeconds);
        steerMotor.update(steerVoltage, dtSeconds);

        // Assume idealized azimuth control - no "twist" force at contact patch from friction or robot motion.
        cursteerAngle = Rotation2d.fromDegrees(steerMotor.getMechanismPosition_Rev() * 360);

        this.azmthVoltageEntry.setDouble(steerVoltage);
        this.wheelVoltageEntry.setDouble(wheelVoltage);
        this.azmthSpeedEntry.setDouble(steerMotor.getMechanismSpeed_RPM());
        this.azmthPosEntry.setDouble(steerMotor.getMechanismPosition_Rev()*360.0);
        this.wheelSpeedEntry.setDouble(driveMotor.wheelSpeed_RPM);
    }

    
    /** Get a vector of the velocity of the module's contact patch moving across the field. */
    Vector2d getModuleRelativeTranslationVelocity(double dtSeconds){
        double xVel = (curModulePose.getTranslation().getX() - prevModulePose.getTranslation().getX())/dtSeconds;
        double yVel = (curModulePose.getTranslation().getY() - prevModulePose.getTranslation().getY())/dtSeconds;
        Vector2d moduleTranslationVec= new Vector2d(xVel,yVel);
        moduleTranslationVec.rotate(-1.0*curModulePose.getRotation().getDegrees());
        return moduleTranslationVec;
    }

    /**
     * Given a net force on a particular module, calculate the friction force
     * generated by the tread interacting with the ground in the direction
     * perpendicular to the wheel's rotation.
     * @param netForce_in
     * @return 
     */
    ForceAtPose2d getCrossTreadFrictionalForce(Force2d netForce_in, double dtSeconds){

        //Project net force onto cross-tread vector
        Vector2d crossTreadUnitVector = new Vector2d(0,1);
        crossTreadUnitVector.rotate(cursteerAngle.getDegrees());
        crossTreadVelMag = getModuleRelativeTranslationVelocity(dtSeconds).dot(crossTreadUnitVector);
        crossTreadForceMag = netForce_in.getVector2d().dot(crossTreadUnitVector);

        Force2d fricForce = new Force2d();
        
        if(Math.abs(crossTreadForceMag) > treadStaticFricForce || Math.abs(crossTreadVelMag) > 0.001){
            // Force is great enough to overcome static friction, or we're already moving
            // In either case, use kinetic frictional model
            crossTreadFricForceMag = -1.0 * Math.signum(crossTreadVelMag) * treadKineticFricForce;
        } else {
            // Static Friction Model
            crossTreadFricForceMag = -1.0 * crossTreadForceMag;
        }
        
        fricForce = new Force2d(crossTreadUnitVector);
        fricForce = fricForce.times(crossTreadFricForceMag);

        return new ForceAtPose2d(fricForce, curModulePose);
    }

        
    /** Gets the modules on-axis (along wheel direction) force, which comes from the rotation of the motor. */
    ForceAtPose2d getWheelMotiveForce(){
        return new ForceAtPose2d(new Force2d(driveMotor.getGroundForce_N(), cursteerAngle), curModulePose);
    }

    /** Set the motion of each module in the field reference frame */
    void setModulePose(Pose2d curPos){
        //Handle init'ing module position history to current on first pass
        if(prevModulePose == null){
            prevModulePose = curPos;
        } else {
            prevModulePose = curModulePose;
        }

        curModulePose = curPos;
    }

    Pose2d getModulePose(){
        return curModulePose;
    }

}
