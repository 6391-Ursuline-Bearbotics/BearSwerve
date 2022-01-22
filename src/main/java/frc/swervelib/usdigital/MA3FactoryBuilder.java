package frc.swervelib.usdigital;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.swervelib.AbsoluteEncoder;
import frc.swervelib.AbsoluteEncoderFactory;

public class MA3FactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;
    private static MA3AbsoluteConfiguration configuration;

    public MA3FactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public AbsoluteEncoderFactory<MA3AbsoluteConfiguration> build() {
        return configuration -> {
            this.configuration = configuration;

            AnalogInput encoder = new AnalogInput(configuration.getId());

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final AnalogInput encoder;

        private EncoderImplementation(AnalogInput encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = (1.0 - encoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI;
            angle += configuration.getOffset();
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }

        @Override
        public void setAbsoluteEncoder(double position, double velocity) {
            //encoder.setSimDevice(device);
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
