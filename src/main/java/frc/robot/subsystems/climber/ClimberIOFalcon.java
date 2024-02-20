package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.MathUtils;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;


public class ClimberIOFalcon implements ClimberIO {
    private TalonFX climberMotor = new TalonFX(16, "CANivore");
    private PositionVoltage control = new PositionVoltage(Rotation2d.fromDegrees(40).getRotations());
    private VoltageOut voltageControl = new VoltageOut(0);
    private DutyCycleEncoder encoder;
    private double lastEncoderAngle = 0;

    private final double errorThreshold = 0.1;

    public ClimberIOFalcon() {
        climberMotor.setInverted(true);
        climberMotor.setPosition(0);
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.currentPosition = climberMotor.getPosition().getValueAsDouble();
        inputs.currentVoltage = climberMotor.getMotorVoltage().getValue();
    }

    public void setVoltage(double voltage) {
        climberMotor.setControl(voltageControl.withOutput(voltage));
    }

    public void setPosition(double position) {
        climberMotor.setPosition(position);
    }
}
