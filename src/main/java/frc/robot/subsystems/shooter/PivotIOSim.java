package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIOSim implements PivotIO {
    private Rotation2d rotation = new Rotation2d();

    public void updateInputs(PivotIOInputs inputs) {
        inputs.atTarget = true;
        inputs.currentAngle = rotation;
    }

    public void setAngle(Rotation2d currentAngle) {
        this.rotation = currentAngle;
    }

    public void setVoltage(double voltage) {
    }

    public void updateAngle(Rotation2d currentAngle) {
        this.rotation = currentAngle;
    }
}
