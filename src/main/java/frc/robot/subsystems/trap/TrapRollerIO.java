package frc.robot.subsystems.trap;

public interface TrapRollerIO {
    
    public void updateInputs(RollerIOInputs inputs);

    public class RollerIOInputs {
        public double speed = 0;
        public double voltage = 0;
        public double current = 0;
        public double motorTemperature = 0;
    }

    public void setVoltage(double voltage);

}
