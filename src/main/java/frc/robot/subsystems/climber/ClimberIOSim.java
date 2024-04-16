package frc.robot.subsystems.climber;

public class ClimberIOSim implements ClimberIO {
    private double position = 0;
    private double voltage = 0;

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.currentPosition = position;
        position += voltage * 0.08;
        inputs.currentVoltage = voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void resetPosition(double position) {
        this.position = position;
    }
}
