package frc.wpiClasses;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;


class SimpleMotorWithMassModel {

    double m_curDisplacementRev;

    FlywheelSim m_fwSim;

    /**
     * So far - this is just a wrapper around FlywheelSim to get position as an output.
     * @param motor Motor driving this mass
     * @param gearing Gearing between motor and controlled mass
     * @param moi Moment of inertia of the controlled mass
     */
    SimpleMotorWithMassModel(DCMotor motor, double gearing, double moi) {
        m_fwSim = new FlywheelSim(motor, gearing, moi);
    }

    void update(double motorVoltage, double dtSeconds) {
        m_fwSim.setInputVoltage(motorVoltage);
        m_fwSim.update(dtSeconds);
        //Add additional state of displacement in a hacky-ish calculation
        m_curDisplacementRev += m_fwSim.getAngularVelocityRPM() / 60.0 * dtSeconds;
    }

    /**
     * Gets the present speed of the rotating mass.
     */
    double getMechanismSpeed_RPM() {
        return m_fwSim.getAngularVelocityRPM();
    }

    /**
     * Returns the present current draw of the mechanism.
     */
    double getCurrent_A() {
        return m_fwSim.getCurrentDrawAmps();
    }

    /**
     * Returns the present displacement in Revolutions.
     */
    double getMechanismPositionRev() {
        return m_curDisplacementRev;
    }
}