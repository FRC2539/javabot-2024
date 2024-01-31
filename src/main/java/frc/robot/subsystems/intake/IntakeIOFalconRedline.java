package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOFalconRedline implements IntakeIO{
    private double chamberSpeed;
    private double rollerSpeed;

    private TalonFX chamberMotor = new TalonFX(IntakeConstants.chamberMotorPort);
    private WPI_TalonSRX rollerMotor = new WPI_TalonSRX(IntakeConstants.rollerMotorPort);

    private AnalogInput rollerSensor = new AnalogInput(IntakeConstants.rollerSensorPort);
    private AnalogInput chamberSensor = new AnalogInput(IntakeConstants.chamberSensorPort);

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.chamberSpeed = chamberSpeed;
        inputs.rollerSpeed = rollerSpeed;

        inputs.chamberSensor = hasChamberPiece();
        inputs.rollerSensor = hasRollerPiece();
    }

    public boolean hasRollerPiece() {
        return rollerSensor.getValue() > 50;
    }

    public boolean hasChamberPiece() {
        return chamberSensor.getValue() > 50;
    }

    public void setRollerSpeed(double speed) {
        rollerSpeed = speed;
        rollerMotor.set(speed);
    }

    public void setChamberSpeed(double speed) {
        chamberSpeed = speed;
        chamberMotor.set(speed);
    }

}
