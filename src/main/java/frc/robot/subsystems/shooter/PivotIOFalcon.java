package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.MathUtils;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class PivotIOFalcon implements PivotIO {
    private TalonFX pivotMotor;
    private MotionMagicDutyCycle control = new MotionMagicDutyCycle(0);
    private VoltageOut voltageControl = new VoltageOut(0);
    private DutyCycleEncoder encoder;
    private double lastEncoderAngle = 0;

    private final double errorThreshold = 0.1;

    public PivotIOFalcon() {
        pivotMotor = new TalonFX(Constants.ShooterConstants.pivotPort, "CANivore");
        FeedbackConfigs feedbackConfigs  = new FeedbackConfigs();
        pivotMotor.getConfigurator().refresh(feedbackConfigs);
        feedbackConfigs.SensorToMechanismRatio = Constants.ShooterConstants.gearRatioPivot;
        pivotMotor.getConfigurator().apply(feedbackConfigs);
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.currentAngle = Rotation2d.fromRotations(pivotMotor.getPosition().getValueAsDouble());
        inputs.atTarget = MathUtils.equalsWithinError(control.Position, pivotMotor.getPosition().getValueAsDouble(), errorThreshold);

        pivotMotor.setPosition(getGripperEncoderAngle());
    }

    public void setAngle(Rotation2d angle) {
        pivotMotor.setControl(control.withPosition(angle.getRotations()));
    }

    public void setVoltage(double voltage) {
        pivotMotor.setControl(voltageControl.withOutput(voltage));
    }

    public void updateAngle(Rotation2d angle) {
        lastEncoderAngle = angle.getRotations() * ShooterConstants.gearRatioPivot;
    }

    private double getGripperEncoderAngle() {
        double currentEncoderAngle = encoder.getAbsolutePosition();
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
