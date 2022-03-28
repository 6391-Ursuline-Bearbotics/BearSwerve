package frc.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyroscope {
  /**
   * Gets the current heading (Yaw) as reported by the gyroscope.
   * @return The Rotation2d value of the heading.
   */
  Rotation2d getGyroHeading();

  /**
  * Sets the gyroscope angle. This can be used to set the direction the robot is currently facing to the
  * 'forwards' direction.
  */
  void zeroGyroscope(double angle);

  /**
   * Sets the simulated gyroscope to a specified angle
   * @param angle Angle to be set in degrees.
   */
  void setAngle(double angle);

  /**
   * Determines if the Gyro is ready to be used.
   * @return True/False if the gyro is ready to be used.
   */
  Boolean getGyroReady();
}
