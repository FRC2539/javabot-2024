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
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import frc.lib.math.MathUtils;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;


public class PivotIOFalcon implements PivotIO {
    private TalonFX pivotMotor;
    private PositionVoltage control = new PositionVoltage(Rotation2d.fromDegrees(40).getRotations());
    private VoltageOut voltageControl = new VoltageOut(0);
    private DutyCycleEncoder encoder;
    private double lastEncoderAngle = 0;

    private final double errorThreshold = 0.1;

    // Aaron's Persistent Addition
    private GenericEntry positionOffset = Shuffleboard.getTab("Position Offset").addPersistent("Position Offset", 45)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    public PivotIOFalcon() {
        pivotMotor = new TalonFX(Constants.ShooterConstants.pivotPort, "CANivore");
        encoder = new DutyCycleEncoder(0);
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
        
        updateAngle(Rotation2d.fromDegrees(47));
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.currentAngle = Rotation2d.fromRotations(pivotMotor.getPosition().getValueAsDouble());
        //inputs.currentAngle = Rotation2d.fromRotations(getGripperEncoderAngle() / Constants.ShooterConstants.gearRatioPivot); //pivotMotor.getPosition().getValueAsDouble());
        inputs.atTarget = MathUtils.equalsWithinError(control.Position, pivotMotor.getPosition().getValueAsDouble(), errorThreshold);

        //pivotMotor.setPosition(getGripperEncoderAngle() / ShooterConstants.gearRatioPivot);
    }

    public void setAngle(Rotation2d angle) {
        pivotMotor.setControl(control.withPosition(angle.getRotations()));
    }

    public void setVoltage(double voltage) {
        pivotMotor.setControl(voltageControl.withOutput(voltage));
    }

    public void updateAngle(Rotation2d angle) {
        lastEncoderAngle = angle.getRotations() * ShooterConstants.gearRatioPivot;
        //encoder.setPositionOffset((0.1938 - (45.0 / 360.0 * Constants.ShooterConstants.gearRatioPivot) + 10) % 1.0);
        encoder.setPositionOffset((0.1938 - (positionOffset.getDouble(45) / 360.0 * Constants.ShooterConstants.gearRatioPivot) + 10) % 1.0);
        pivotMotor.setPosition(getGripperEncoderAngle() / ShooterConstants.gearRatioPivot);

        System.out.println(encoder.getPositionOffset());
    }

    private double getGripperEncoderAngle() {
        double currentEncoderAngle = encoder.getAbsolutePosition() - encoder.getPositionOffset();
        // ready position is approximately 90 degrees default starting position of the gripper is 90 degrees. When
        // booting, make sure gripper is between -90 and 270 degrees. (Not backwards.)
        double result = MathUtils.accomidateOverflow(
                lastEncoderAngle,
                currentEncoderAngle,
                1);
        lastEncoderAngle = result;
        return result;
    }


}
