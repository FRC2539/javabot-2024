package frc.robot.subsystems.shooter;

public class RollerIOSim implements RollerIO{
    private double RPM;

    public RollerIOSim() {};

    public void updateInputs(RollerIOInputs inputs) {
        inputs.speed = RPM;
    }

    public void setSpeed(double RPM) {
        this.RPM = RPM;
    }

}
