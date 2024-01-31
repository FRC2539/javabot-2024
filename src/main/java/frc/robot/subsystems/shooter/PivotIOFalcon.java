package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.math.MathUtils;
import frc.robot.Constants;

public class PivotIOFalcon implements PivotIO {
    private TalonFX pivotMotor;
    private MotionMagicDutyCycle control = new MotionMagicDutyCycle(0);

    private final double errorThreshold = 0.1;

    public PivotIOFalcon() {
        pivotMotor = new TalonFX(Constants.ShooterConstants.pnematics1Forward, "canivore");
    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.currentAngle = pivotMotor.getPosition().getValueAsDouble() * Math.PI * 2;
        inputs.atTarget = MathUtils.equalsWithinError(control.Position, pivotMotor.getPosition().getValueAsDouble(), errorThreshold);
    }

    public void setAngle(double angle) {
        pivotMotor.setControl(control.withPosition(angle / (2 * Math.PI)));
    }
}
