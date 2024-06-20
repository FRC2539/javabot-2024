package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PivotIO {

    public void updateInputs(PivotIOInputs inputs);

    public class PivotIOInputs {
        public Rotation2d currentAngle = new Rotation2d();
        public boolean atTarget = false;
        public boolean isEncoderConnected = false;
    }

    public void setAngle(Rotation2d targetAngle);

    public void setVoltage(double voltage);

    public void updateAngle(Rotation2d targetAngle);

    default public Rotation2d getGripperEncoderAngle() {
        return new Rotation2d();
    }
}
