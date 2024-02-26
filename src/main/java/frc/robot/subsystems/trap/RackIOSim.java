package frc.robot.subsystems.trap;


public class RackIOSim implements RackIO {
    private double position = 0;

    public void updateInputs(RackIOInputs inputs) {
        inputs.atTarget = true;
        inputs.position = position;
    }

    public void setPosition(double position) {
        this.position = position;
    }

    public void setVoltage(double voltage) {
    }

    public void zeroPosition() {
        this.position = 0;
    }
}
