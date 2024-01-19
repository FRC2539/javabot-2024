package frc.robot.subsystems.shooter;

public interface RollerIO {
    
    public void updateInputs(RollerIOInputs inputs);

    public class RollerIOInputs {
        public double speed = 0;
        public double voltage = 0;
        public double current = 0;
        public double motorTemperature = 0;
    }

    public void setSpeed(double speed);

}
