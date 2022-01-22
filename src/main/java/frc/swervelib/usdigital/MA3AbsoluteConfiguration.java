package frc.swervelib.usdigital;

public class MA3AbsoluteConfiguration {
    private final int id;
    private final double offset;

    public MA3AbsoluteConfiguration(int id, double offset) {
        this.id = id;
        this.offset = offset;
    }

    public int getId() {
        return id;
    }

    public double getOffset() {
        return offset;
    }
}
