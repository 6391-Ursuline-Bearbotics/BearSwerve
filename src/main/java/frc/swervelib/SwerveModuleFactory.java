package frc.swervelib;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveModuleFactory<DriveConfiguration, SteerConfiguration> {
    private final ModuleConfiguration moduleConfiguration;
    private final DriveControllerFactory<?, DriveConfiguration> driveControllerFactory;
    private final SteerControllerFactory<?, SteerConfiguration> steerControllerFactory;

    public SwerveModuleFactory(ModuleConfiguration moduleConfiguration,
                               DriveControllerFactory<?, DriveConfiguration> driveControllerFactory,
                               SteerControllerFactory<?, SteerConfiguration> steerControllerFactory) {
        this.moduleConfiguration = moduleConfiguration;
        this.driveControllerFactory = driveControllerFactory;
        this.steerControllerFactory = steerControllerFactory;
    }

    public SwerveModule create(DriveConfiguration driveConfiguration, SteerConfiguration steerConfiguration, String namePrefix) {
        var driveController = driveControllerFactory.create(driveConfiguration, moduleConfiguration);
        var steerController = steerControllerFactory.create(steerConfiguration, moduleConfiguration);

        return new ModuleImplementation(driveController, steerController, namePrefix);
    }

    public SwerveModule create(ShuffleboardLayout container, DriveConfiguration driveConfiguration, SteerConfiguration steerConfiguration, String namePrefix) {
        var driveController = driveControllerFactory.create(
                container,
                driveConfiguration,
                moduleConfiguration
        );
        var steerContainer = steerControllerFactory.create(
                container,
                steerConfiguration,
                moduleConfiguration
        );

        return new ModuleImplementation(driveController, steerContainer,namePrefix);
    }

    private class ModuleImplementation implements SwerveModule {
        private final DriveController driveController;
        private final SteerController steerController;

        
        private ShuffleboardTab tab = Shuffleboard.getTab("SwerveDt");
        private NetworkTableEntry driveVoltageCmdEntry;
        private NetworkTableEntry steerAngleCmdEntry;

        private ModuleImplementation(DriveController driveController, SteerController steerController, String namePrefix) {
            this.driveController = driveController;
            this.steerController = steerController;

            this.driveVoltageCmdEntry = tab.add(namePrefix + "Wheel Voltage Cmd V", 0).getEntry();
            this.steerAngleCmdEntry = tab.add(namePrefix + "Azmth Des Angle Deg", 0).getEntry();
    
    
        }

        @Override
        public void resetWheelEncoder() {
            driveController.resetEncoder();
        }

        @Override
        public double getDriveVelocity() {
            return driveController.getStateVelocity();
        }

        @Override
        public double getSteerAngle() {
            return steerController.getStateAngle();
        }

        @Override
        public ModuleConfiguration getModuleConfiguration() {
            return moduleConfiguration;
        }

        @Override
        public DriveController getDriveController() {
            return driveController;
        }

        @Override
        public SteerController getSteerController() {
            return steerController;
        }

        @Override
        public AbsoluteEncoder getAbsoluteEncoder() {
            return steerController.getAbsoluteEncoder();
        }

        
        @Override
        public void set(double driveVoltage, double steerAngle) {
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            double difference = steerAngle - getSteerAngle();
            // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
            if (difference >= Math.PI) {
                steerAngle -= 2.0 * Math.PI;
            } else if (difference < -Math.PI) {
                steerAngle += 2.0 * Math.PI;
            }
            difference = steerAngle - getSteerAngle(); // Recalculate difference

            // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
            // movement of the module is less than 90 deg
            if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
                // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
                steerAngle += Math.PI;
                driveVoltage *= -1.0;
            }

            // Put the target angle back into the range [0, 2pi)
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            driveController.setReferenceVoltage(driveVoltage);
            steerController.setReferenceAngle(steerAngle);

            this.driveVoltageCmdEntry.setDouble(driveVoltage);
            this.steerAngleCmdEntry.setDouble(steerAngle*180/Math.PI);
        }
    }
}
