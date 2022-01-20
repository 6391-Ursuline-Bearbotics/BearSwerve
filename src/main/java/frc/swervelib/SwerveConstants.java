package frc.swervelib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveConstants {
    public static double MAX_FWD_REV_SPEED_MPS;
    public static double MAX_STRAFE_SPEED_MPS;
    public static double MAX_ROTATE_SPEED_RAD_PER_SEC;
    public static double MAX_VOLTAGE;
    public static Pose2d DFLT_START_POSE;
    
    public static double THETACONTROLLERkP;
    public static double TRAJECTORYXkP;
    public static double TRAJECTORYYkP;
    public static TrapezoidProfile.Constraints THETACONTROLLERCONSTRAINTS;

    public static double TRACKWIDTH_METERS;
    public static double TRACKLENGTH_METERS;
    public static double MASS_kg;
    public static double MOI_KGM2;
    public static SwerveDriveKinematics KINEMATICS;

    public static PIDController XPIDCONTROLLER = new PIDController(TRAJECTORYXkP, 0, 0);
    public static PIDController YPIDCONTROLLER = new PIDController(TRAJECTORYYkP, 0, 0);
}
