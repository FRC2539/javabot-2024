package frc.robot.subsystems.trap;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.MathUtils;
import frc.robot.Constants;


public class RackIONeo550 implements RackIO {
    private CANSparkMax neo550;
    private PositionVoltage control = new PositionVoltage(Rotation2d.fromDegrees(40).getRotations());
    private SparkPIDController pidController;

    private final double errorThreshold = 0.1;

    private boolean shutdown = false;

    public RackIONeo550() {
        neo550 = new CANSparkMax(Constants.ShooterConstants.pivotPort, MotorType.kBrushless);
        
        neo550.setSmartCurrentLimit(40);
        neo550.setSecondaryCurrentLimit(45);

        neo550.getEncoder().setPosition(0);
        neo550.setSoftLimit(SoftLimitDirection.kReverse, 0);
        neo550.setSoftLimit(SoftLimitDirection.kForward, 10);

        pidController = neo550.getPIDController();

        pidController.setP(0.1);
        pidController.setI(0);
        pidController.setD(1);
        pidController.setOutputRange(-1,1);

        neo550.burnFlash();
    }

    public void updateInputs(RackIOInputs inputs) {
        inputs.position = neo550.getEncoder().getPosition();
        //inputs.currentAngle = Rotation2d.fromRotations(getGripperEncoderAngle() / Constants.ShooterConstants.gearRatioPivot); //pivotMotor.getPosition().getValueAsDouble());
        inputs.atTarget = MathUtils.equalsWithinError(control.Position, inputs.position, errorThreshold);

        inputs.temperature = neo550.getMotorTemperature();

        if (inputs.temperature > 40) {
            shutdown = true;
            neo550.stopMotor();
        } else if (inputs.temperature < 30) {
            shutdown = false;
        }
        //pivotMotor.setPosition(getGripperEncoderAngle() / ShooterConstants.gearRatioPivot);
    }

    public void setPosition(double position) {
        if (shutdown) return;
        pidController.setReference(position, ControlType.kPosition);
    }

    public void setVoltage(double voltage) {
        if (shutdown) return;
        neo550.setVoltage(voltage);
    }
}
