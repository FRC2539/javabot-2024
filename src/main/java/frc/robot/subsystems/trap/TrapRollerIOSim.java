package frc.robot.subsystems.trap;

public class TrapRollerIOSim implements TrapRollerIO{
    private double voltage;

    public TrapRollerIOSim() {};

    public void updateInputs(RollerIOInputs inputs) {
        inputs.speed = voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

}
