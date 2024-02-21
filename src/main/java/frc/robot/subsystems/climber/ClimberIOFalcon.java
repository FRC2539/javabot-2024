package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;


public class ClimberIOFalcon implements ClimberIO {
    private TalonFX climberMotor = new TalonFX(16, "CANivore"); 
    private VoltageOut voltageControl = new VoltageOut(0);

    public ClimberIOFalcon() {
        climberMotor.setInverted(true);
        climberMotor.setPosition(0);
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.currentPosition = climberMotor.getPosition().getValueAsDouble();
        inputs.currentVoltage = climberMotor.getMotorVoltage().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        climberMotor.setControl(voltageControl.withOutput(voltage));
    }

    public void setPosition(double position) {
        climberMotor.setPosition(position);
    }
}
