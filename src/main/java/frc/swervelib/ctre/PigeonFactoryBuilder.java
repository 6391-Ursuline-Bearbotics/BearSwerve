package frc.swervelib.ctre;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.swervelib.Gyroscope;

public class PigeonFactoryBuilder {
    private static BasePigeonSimCollection pigeonSim;

    private static double gyroOffset = 0.0;

    public Gyroscope build(WPI_PigeonIMU pigeon) {
        return new GyroscopeImplementation(pigeon);
    }

    private static class GyroscopeImplementation implements Gyroscope {
        private final WPI_PigeonIMU pigeon;

        private GyroscopeImplementation(WPI_PigeonIMU pigeon) {
            this.pigeon = pigeon;
            pigeonSim = pigeon.getSimCollection();
        }

        @Override
        public Rotation2d getGyroHeading() {
            return Rotation2d.fromDegrees(pigeon.getFusedHeading() + gyroOffset);
        }

        @Override
        public Boolean getGyroReady() {
            return pigeon.getState().equals(PigeonState.Ready);
        }

        @Override
        public void zeroGyroscope(double angle) {
            gyroOffset = angle - getGyroHeading().getDegrees();
        }

        @Override
        public void setAngle(double angle) {
            pigeonSim.setRawHeading(angle);
        }
    }
}
