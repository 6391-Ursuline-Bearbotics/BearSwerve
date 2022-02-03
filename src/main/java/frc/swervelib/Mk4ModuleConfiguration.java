package frc.swervelib;

import java.util.Objects;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Additional Mk4 module configuration parameters.
 * <p>
 * The configuration parameters here are used to customize the behavior of the Mk4 swerve module.
 * Each setting is initialized to a default that should be adequate for most use cases.
 */
public class Mk4ModuleConfiguration {
    private double nominalVoltage = 12.0;
    private double driveCurrentLimit = 80.0;
    private double steerCurrentLimit = 20.0;

    private DCMotor driveMotor;
    private DCMotor steerMotor;

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    public double getDriveCurrentLimit() {
        return driveCurrentLimit;
    }

    public void setDriveCurrentLimit(double driveCurrentLimit) {
        this.driveCurrentLimit = driveCurrentLimit;
    }

    public double getSteerCurrentLimit() {
        return steerCurrentLimit;
    }

    public void setSteerCurrentLimit(double steerCurrentLimit) {
        this.steerCurrentLimit = steerCurrentLimit;
    }

    /**
     * @return Gets the type of the drive motor.
     */
    public DCMotor getDriveMotor() {
        return driveMotor;
    }

    /**
     * @return Gets the type of the steer motor.
     */
    public DCMotor getSteerMotor() {
        return steerMotor;
    }

    /**
     * Sets the type of the drive motor.
     * @param motor The DCMotor to be used as the Drive Motor
     */
    public void setDriveMotor(DCMotor motor) {
        driveMotor = motor;
    }

    /**
     * Sets the type of the steer motor.
     * @param motor The DCMotor to be used as the Steer Motor
     */
    public void setSteerMotor(DCMotor motor) {
        steerMotor = motor;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Mk4ModuleConfiguration that = (Mk4ModuleConfiguration) o;
        return Double.compare(that.getNominalVoltage(), getNominalVoltage()) == 0 && Double.compare(that.getDriveCurrentLimit(), getDriveCurrentLimit()) == 0 && Double.compare(that.getSteerCurrentLimit(), getSteerCurrentLimit()) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getNominalVoltage(), getDriveCurrentLimit(), getSteerCurrentLimit());
    }

    @Override
    public String toString() {
        return "Mk4ModuleConfiguration{" +
                "nominalVoltage=" + nominalVoltage +
                ", driveCurrentLimit=" + driveCurrentLimit +
                ", steerCurrentLimit=" + steerCurrentLimit +
                ", steerMotor=" + steerMotor +
                ", driveMotor=" + driveMotor +
                '}';
    }
}
