package frc.wpiClasses;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import org.ejml.simple.SimpleMatrix;

public class Force2d {
  Matrix<N2, N1> m_matrix;

  /**
   * Constructs a Force2d with X and Y components equal to zero.
   */
  public Force2d() {
    this(0.0, 0.0);
  }

  /**
   * Constructs a Force2d with the X and Y components equal to the
   * provided values.
   *
   * @param x The x component of the force.
   * @param y The y component of the force.
   */
  public Force2d( double x, double y) {
    m_matrix = new Matrix<>(new SimpleMatrix(2, 1));
    m_matrix.set(0, 0, x);
    m_matrix.set(1, 0, y);
  }

  /**
   * Constructs a Force2d with the provided force magnitude and angle. This is
   * essentially converting from polar coordinates to Cartesian coordinates.
   *
   * @param mag The magnititude of the force
   * @param angle    The angle from the x-axis to the force vector.
   */
  public Force2d(double mag, Rotation2d angle) {
    this(mag * angle.getCos(), mag * angle.getSin());
  }

  /**
   * Constructs a Force2d with the provided 2-element column matrix as the x/y components.
   *
   * @param m 2 row, 1 column input matrix
   */
  public Force2d(Matrix<N2, N1> m) {
    m_matrix = m;
  }

  /**
   * Constructs a Force2d with the provided vector assumed to represent the force.
   *
   * @param forceVec vector which represents some force in two dimensions
   */
  public Force2d(Vector2d forceVec) {
    this(forceVec.x, forceVec.y);
  }

  /**
   * @return Returns the X component of the force.
   */
  public double getX() {
    return m_matrix.get(0, 0);
  }

  /**
   * @return Returns the Y component of the force.
   */
  public double getY() {
    return m_matrix.get(1, 0);
  }

  /**
   * @return Returns the norm, or distance from the origin to the force.
   */
  public double getNorm() {
    return m_matrix.normF();
  }

  /**
   * @return Gets a unit vector in the direction this force points.
   */
  public Vector2d getUnitVector() {
    return new Vector2d(this.getX() / this.getNorm(), this.getY() / this.getNorm());
  }

  /**
   * Applies a rotation to the force in 2d space.
   *
   * <p>This multiplies the force vector by a counterclockwise rotation
   * matrix of the given angle.
   * [x_new]   [other.cos, -other.sin][x]
   * [y_new] = [other.sin,  other.cos][y]
   *
   * <p>For example, rotating a Force2d of {2, 0} by 90 degrees will return a
   * Force2d of {0, 2}.
   *
   * @param angle The rotation to rotate the force by.
   * @return The new rotated force.
   */
  public Force2d rotateBy(Rotation2d angle) {
    return new Force2d(
            this.getX() * angle.getCos() - this.getY() * angle.getSin(),
            this.getX() * angle.getSin() + this.getY() * angle.getCos()
    );
  }

  /**
   * Adds two forces in 2d space and returns the sum. This is similar to
   * vector addition.
   *
   * <p>For example, Force2d{1.0, 2.5} + Force2d{2.0, 5.5} =
   * Force2d{3.0, 8.0}
   *
   * @param other The force to add.
   * @return The sum of the forces.
   */
  public Force2d plus(Force2d other) {
    return new Force2d(this.m_matrix.plus(other.m_matrix));
  }

  /**
   * Accumulates another force into this force. Returns nothing, acts "in-place" on this force.
   * @param other The force to add.
   */
  public void accum(Force2d other) {
    this.m_matrix = this.m_matrix.plus(other.m_matrix);
  }

  /**
   * Subtracts the other force from the other force and returns the
   * difference.
   *
   * <p>For example, Force2d{5.0, 4.0} - Force2d{1.0, 2.0} =
   * Force2d{4.0, 2.0}
   *
   * @param other The force to subtract.
   * @return The difference between the two forces.
   */
  public Force2d minus(Force2d other) {
    return new Force2d(this.m_matrix.minus(other.m_matrix));
  }

  /**
   * Returns the inverse of the current force. This is equivalent to
   * rotating by 180 degrees, flipping the point over both axes, or simply
   * negating both components of the force.
   *
   * @return The inverse of the current force.
   */
  public Force2d unaryMinus() {
    return new Force2d(this.m_matrix.times(-1.0));
  }

  /**
   * Multiplies the force by a scalar and returns the new force.
   *
   * <p>For example, Force2d{2.0, 2.5} * 2 = Force2d{4.0, 5.0}
   *
   * @param scalar The scalar to multiply by.
   * @return The scaled force.
   */
  public Force2d times(double scalar) {
    return new Force2d(this.m_matrix.times(scalar));
  }

  /**
   * Divides the force by a scalar and returns the new force.
   *
   * <p>For example, Force2d{2.0, 2.5} / 2 = Force2d{1.0, 1.25}
   *
   * @param scalar The scalar to multiply by.
   * @return The reference to the new mutated object.
   */
  public Force2d div(double scalar) {
    return new Force2d(this.m_matrix.div(scalar));
  }

  /**
   * @return Creates a Vector2d object from the force this object represents.
   */
  public Vector2d getVector2d() {
    return new Vector2d(this.getX(), this.getY());
  }

  @Override
  public String toString() {
    return String.format("Force2d(X: %.2f, Y: %.2f)", this.getX(), this.getY());
  }

  /**
   * Checks equality between this Force2d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Force2d) {
      return this.m_matrix.isEqual(((Force2d) obj).m_matrix, 1E-9);
    } else {
      return false;
    }
  }
}
