package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.framework.motor.MotorIOTalonFX;
import frc.robot.Constants;

public class PivotIOFalconTest extends MotorIOTalonFX implements PivotIO {
    private boolean isEncoderConnected = false;

    protected DutyCycleEncoder encoder;

    public PivotIOFalconTest(int port, int encoderPort) {
        super(port, "CANivore");

        motor.setInverted(true);
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        motor.getConfigurator().refresh(feedbackConfigs);
        feedbackConfigs.SensorToMechanismRatio = Constants.ShooterConstants.gearRatioPivot * 16;
        motor.getConfigurator().apply(feedbackConfigs);
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 240;
        slot0Configs.kS = 0.3;
        motor.getConfigurator().apply(slot0Configs);
    }

    public void update() {
        super.update();
        isEncoderConnected = encoder.isConnected();
        motor.setPosition(getGripperEncoderAngle().getRotations());
    }

    public boolean isEncoderConnected() {
        return isEncoderConnected;
    }

    protected Rotation2d getGripperEncoderAngle() {
        double currentEncoderAngle = encoder.getAbsolutePosition() * 1.16 - 0.575628;
        return Rotation2d.fromRotations(currentEncoderAngle);
    }
}
