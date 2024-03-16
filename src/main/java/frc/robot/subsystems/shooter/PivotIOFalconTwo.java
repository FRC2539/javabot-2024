package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.lib.math.MathUtils;
import frc.robot.Constants;


public class PivotIOFalconTwo implements PivotIO {
    private TalonFX pivotMotor;
    private PositionVoltage control = new PositionVoltage(Rotation2d.fromDegrees(40).getRotations());
    private VoltageOut voltageControl = new VoltageOut(0);
    private DutyCycleEncoder encoder;

    private final double errorThreshold = 0.1;

    // Aaron's Persistent Addition
    private GenericEntry positionOffset = Shuffleboard.getTab("Position Offset").addPersistent("Position Offset", 45)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    public PivotIOFalconTwo() {
        pivotMotor = new TalonFX(Constants.ShooterConstants.pivotPort, "CANivore");
        encoder = new DutyCycleEncoder(9);
        encoder.setPositionOffset(0);
        pivotMotor.setInverted(true);
        FeedbackConfigs feedbackConfigs  = new FeedbackConfigs();
        pivotMotor.getConfigurator().refresh(feedbackConfigs);
        feedbackConfigs.SensorToMechanismRatio = Constants.ShooterConstants.gearRatioPivot * 16;
        pivotMotor.getConfigurator().apply(feedbackConfigs);
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 240;
        slot0Configs.kS = 0.3;
        pivotMotor.getConfigurator().apply(slot0Configs);
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.currentAngle = getGripperEncoderAngle();
        //inputs.currentAngle = Rotation2d.fromRotations(getGripperEncoderAngle() / Constants.ShooterConstants.gearRatioPivot); //pivotMotor.getPosition().getValueAsDouble());
        inputs.atTarget = MathUtils.equalsWithinError(control.Position, inputs.currentAngle.getRotations(), errorThreshold);
        pivotMotor.setPosition(inputs.currentAngle.getRotations());

        inputs.isEncoderConnected = encoder.isConnected();
        //pivotMotor.setPosition(getGripperEncoderAngle() / ShooterConstants.gearRatioPivot);
    }

    public void setAngle(Rotation2d angle) {
        pivotMotor.setControl(control.withPosition(angle.getRotations()));
    }

    public void setVoltage(double voltage) {
        pivotMotor.setControl(voltageControl.withOutput(voltage));
    }

    public void updateAngle(Rotation2d angle) {
        encoder.getAbsolutePosition();
        System.out.println(encoder.getAbsolutePosition());
    }

    private Rotation2d getGripperEncoderAngle() {
        double currentEncoderAngle = (encoder.getAbsolutePosition() - 0.60399) * (29.0 / 25.0) + 45.0 / 360.0;
        return Rotation2d.fromRotations(currentEncoderAngle);
    }


}
