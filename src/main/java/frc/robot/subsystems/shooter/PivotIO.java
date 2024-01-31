package frc.robot.subsystems.shooter;

public interface PivotIO {

    public void updateInputs(PivotIOInputs inputs);

    public class PivotIOInputs {
        public double currentAngle = 0;
        public boolean atTarget = false;
    }

    public void setAngle(double targetAngle);
}
