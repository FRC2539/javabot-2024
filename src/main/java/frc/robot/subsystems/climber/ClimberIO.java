package frc.robot.subsystems.climber;

public interface ClimberIO {

    public void updateInputs(ClimberIOInputs inputs);

    public class ClimberIOInputs {
        public double currentPosition = 0;
        public double currentVoltage = 0; 
    }

    public void setVoltage(double voltage);

    public void setPosition(double position);
}
