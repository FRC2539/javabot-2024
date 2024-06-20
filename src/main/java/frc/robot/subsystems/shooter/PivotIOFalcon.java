package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.MathUtils;
import frc.robot.Constants;

public class PivotIOFalcon implements PivotIO {
    private TalonFX pivotMotor;
    private PositionVoltage control =
            new PositionVoltage(Rotation2d.fromDegrees(40).getRotations());
    private VoltageOut voltageControl = new VoltageOut(0);
    private DutyCycleEncoder encoder;

    private final double errorThreshold = 0.1;

    public PivotIOFalcon() {
        pivotMotor = new TalonFX(Constants.ShooterConstants.pivotPort, "CANivore");
        encoder = new DutyCycleEncoder(9);
        encoder.setPositionOffset(0);
        pivotMotor.setInverted(true);
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        pivotMotor.getConfigurator().refresh(feedbackConfigs);
        feedbackConfigs.SensorToMechanismRatio = Constants.ShooterConstants.gearRatioPivot * 16;
        pivotMotor.getConfigurator().apply(feedbackConfigs);
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 240;
        //slot0Configs.kS = 0.4;
        //slot0Configs.kD = 50;
        pivotMotor.getConfigurator().apply(slot0Configs);
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        pivotMotor.getConfigurator().refresh(currentLimitsConfigs);
        currentLimitsConfigs.StatorCurrentLimit = 80;
        currentLimitsConfigs.SupplyCurrentLimit = 90;
        pivotMotor.getConfigurator().apply(currentLimitsConfigs);
        pivotMotor.setPosition(Rotation2d.fromDegrees(56.5).getRotations());
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.currentAngle = Rotation2d.fromRotations(pivotMotor.getPosition().getValue());
        inputs.atTarget =
                MathUtils.equalsWithinError(control.Position, inputs.currentAngle.getRotations(), errorThreshold);
        //pivotMotor.setPosition(getGripperEncoderAngle().getRotations());
        inputs.isEncoderConnected = encoder.isConnected();
    }

    public void setAngle(Rotation2d angle) {
        pivotMotor.setControl(control.withPosition(angle.getRotations()));
    }

    public void setVoltage(double voltage) {
        pivotMotor.setControl(voltageControl.withOutput(voltage));
    }

    public void updateAngle(Rotation2d angle) {
        encoder.getAbsolutePosition();
        pivotMotor.setPosition(getGripperEncoderAngle().getRotations());
        System.out.println(encoder.getAbsolutePosition());
    }

    public Rotation2d getGripperEncoderAngle() {
        double currentEncoderAngle = encoder.getAbsolutePosition() * 1.16 - 0.575628;
        return Rotation2d.fromRotations(currentEncoderAngle);
    }
}
