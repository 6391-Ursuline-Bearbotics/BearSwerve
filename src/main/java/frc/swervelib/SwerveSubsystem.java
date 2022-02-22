// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervelib;

import frc.wpiClasses.QuadSwerveSim;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveModuleState[] states;

  private ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);
  public SwerveDrivetrainModel dt;
  public SwerveDriveOdometry m_odometry;

  public SwerveSubsystem(SwerveDrivetrainModel dt) {
    this.dt = dt;
    modules = dt.getRealModules();
    m_odometry = new SwerveDriveOdometry(SwerveConstants.KINEMATICS, dt.getGyroscopeRotation());
  }

  @Override
  public void periodic() {
    states = dt.getSwerveModuleStates();

    if (states != null) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_FWD_REV_SPEED_MPS);

      modules.get(0).set(states[0].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[0].angle.getRadians());
      modules.get(1).set(states[1].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[1].angle.getRadians());
      modules.get(2).set(states[2].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[2].angle.getRadians());
      modules.get(3).set(states[3].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[3].angle.getRadians());
    }

    states[0].speedMetersPerSecond = Math.abs(modules.get(0).getDriveVelocity());
    states[1].speedMetersPerSecond = Math.abs(modules.get(1).getDriveVelocity());
    states[2].speedMetersPerSecond = Math.abs(modules.get(2).getDriveVelocity());
    states[3].speedMetersPerSecond = Math.abs(modules.get(3).getDriveVelocity());
    m_odometry.update(dt.getGyroscopeRotation(), states);

    dt.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    dt.update(DriverStation.isDisabled(), 13.2);
  }
}
