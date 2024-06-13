package frc.robot.subsystems.shamper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.math.MathUtils;
import frc.robot.Constants;


public class ShamperIONeo550 implements ShamperIO {
    private CANSparkMax neo550;
    private SparkPIDController pidController;
    private double goalPosition;

    private final double errorThreshold = 0.1;

    private boolean shutdown = false;

    public ShamperIONeo550() {
        neo550 = new CANSparkMax(Constants.ShamperConstants.shamperMotorPort, MotorType.kBrushless);

        neo550.setSmartCurrentLimit(30);
        neo550.setSecondaryCurrentLimit(35);

        neo550.getEncoder().setPosition(0);
        neo550.setSoftLimit(SoftLimitDirection.kReverse, 0.0f);
        neo550.setSoftLimit(SoftLimitDirection.kForward, 16.0f);

        pidController = neo550.getPIDController();

        pidController.setP(1);
        pidController.setI(0);
        pidController.setD(5);
        pidController.setFF(0.001);

        neo550.setIdleMode(IdleMode.kBrake);

        neo550.setInverted(true);

        pidController.setOutputRange(-.35,1);

        neo550.burnFlash();
    }

    public void updateInputs(RackIOInputs inputs) {
        inputs.position = neo550.getEncoder().getPosition();
        //inputs.currentAngle = Rotation2d.fromRotations(getGripperEncoderAngle() / Constants.ShooterConstants.gearRatioPivot); //pivotMotor.getPosition().getValueAsDouble());
        inputs.atTarget = MathUtils.equalsWithinError(goalPosition, inputs.position, errorThreshold);

        inputs.temperature = neo550.getMotorTemperature();

        inputs.current = neo550.getOutputCurrent();

        inputs.voltage = neo550.getAppliedOutput();

        if (inputs.temperature > 65) {
            shutdown = true;
            neo550.stopMotor();
        } else if (inputs.temperature < 63) {
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

    public void zeroPosition() {
        neo550.getEncoder().setPosition(0);
    }
}