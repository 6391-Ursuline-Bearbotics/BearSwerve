package frc.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    ModuleConfiguration getModuleConfiguration();

    DriveController getDriveController();

    SteerController getSteerController();

    AbsoluteEncoder getAbsoluteEncoder();

    void resetWheelEncoder();

    void set(double driveVoltage, double steerAngle);
}
