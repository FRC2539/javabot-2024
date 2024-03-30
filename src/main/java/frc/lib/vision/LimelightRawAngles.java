package frc.lib.vision;

public record LimelightRawAngles(double tx, double ty, double ta, double timestamp) {
    public LimelightRawAngles(double tx, double ty) {
        this(tx, ty, 0.0);
    }

    public LimelightRawAngles(double tx, double ty, double ta) {
        this(ty, tx, ta, 0.0);
    }
}
