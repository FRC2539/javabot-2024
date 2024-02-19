package frc.robot.subsystems.trap;

public interface RackIO {

    public void updateInputs(RackIOInputs inputs);

    public class RackIOInputs {
        public double position = 0;
        public boolean atTarget = false;
        public double temperature = 0;
    }

    public void setPosition(double targetAngle);

    public void setVoltage(double voltage);
}
