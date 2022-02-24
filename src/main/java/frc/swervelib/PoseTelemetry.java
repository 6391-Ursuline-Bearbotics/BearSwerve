package frc.swervelib;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.wpiClasses.QuadSwerveSim;

public class PoseTelemetry {
    QuadSwerveSim swerveDt;
    SwerveDrivePoseEstimator m_poseEstimator;
    private static Field2d field = new Field2d();

    // Pose at the end of the last update
    Pose2d endPose = SwerveConstants.DFLT_START_POSE;

    // Desired Position says where path planning logic wants the
    // robot to be at any given time.
    Pose2d desiredPose = new Pose2d();

    // Estimated position says where you think your robot is at
    // Based on encoders, motion, vision, etc.
    Pose2d estimatedPose = new Pose2d();

    // Actual position defines wherever the robot is actually at
    // at any time. It is unknowable in real life. The simulation
    // generates this as its primary output.
    Pose2d actualPose = new Pose2d();

    public PoseTelemetry(QuadSwerveSim swerveDt, SwerveDrivePoseEstimator m_poseEstimator) {
        this.swerveDt = swerveDt;
        this.m_poseEstimator = m_poseEstimator;
        SmartDashboard.putData("Field", field);
        field.setRobotPose(SwerveConstants.DFLT_START_POSE);
    }

    public void setActualPose(Pose2d act) {
        actualPose = act;
    }

    public void update(double time) {
        // Check if the user moved the robot with the Field2D
        // widget, and reset the model if so.
        Pose2d startPose = field.getRobotPose();
        Transform2d deltaPose = startPose.minus(endPose);
        if(deltaPose.getRotation().getDegrees() > 0.01 || deltaPose.getTranslation().getNorm() > 0.01){
            swerveDt.modelReset(startPose);
        }

        if (RobotBase.isSimulation()) {
            //field.getObject("Robot").setPose(actualPose);
        }
        field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        endPose = field.getRobotPose();
    }

    public Pose2d getFieldPose() {
        return field.getRobotObject().getPose();
    }

    public Field2d getField() {
        return field;
    }

}