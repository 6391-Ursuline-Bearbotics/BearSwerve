package frc.swervelib;

public class SwerveInput {
    public double m_translationX;
    public double m_translationY;
    public double m_rotation;

    public SwerveInput(double translationX, double translationY, double rotation) {
        this.m_translationX = translationX;
        this.m_translationY = translationY;
        this.m_rotation = rotation;
    }
}
