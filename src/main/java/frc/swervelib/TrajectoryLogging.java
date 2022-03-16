// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervelib;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TrajectoryLogging extends CommandBase {
  private final Timer m_timer = new Timer();
  PathPlannerTrajectory trajectory;
  Supplier<Pose2d> poseSupplier;
  
  /** Creates a new TrajectoryLogging. */
  public TrajectoryLogging(PathPlannerTrajectory trajectory, Supplier<Pose2d> pose) {
    this.trajectory = trajectory;
    this.poseSupplier = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PathPlannerState state = (PathPlannerState) trajectory.sample(m_timer.get());
    SmartDashboard.putNumber("Desired X", state.poseMeters.getX());
    SmartDashboard.putNumber("Desired Y", state.poseMeters.getY());
    SmartDashboard.putNumber("Desired Rotation", state.holonomicRotation.getDegrees());
    SmartDashboard.putNumber("Current X", poseSupplier.get().getX());
    SmartDashboard.putNumber("Current Y", poseSupplier.get().getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
