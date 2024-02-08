package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOFalconRedline implements IntakeIO{
    private double chamberSpeed;
    private double rollerSpeed;

    private final double percentMaxOutput = 1;

    private TalonFX chamberMotor = new TalonFX(IntakeConstants.chamberMotorPort, "CANivore");
    private TalonFX rollerMotor = new TalonFX(IntakeConstants.rollerMotorPort, "CANivore");

    private AnalogInput rollerSensor = new AnalogInput(IntakeConstants.rollerSensorPort);
    private AnalogInput chamberSensor = new AnalogInput(IntakeConstants.chamberSensorPort);

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.chamberSpeed = chamberSpeed;
        inputs.rollerSpeed = rollerSpeed;

        inputs.chamberSensor = hasChamberPiece();
        inputs.rollerSensor = hasRollerPiece();
    }

    public boolean hasRollerPiece() {
        return false; //rollerSensor.getValue() > 50;
    }

    public boolean hasChamberPiece() {
        return false; //chamberSensor.getValue() > 50;
    }

    public void setRollerSpeed(double speed) {
        rollerSpeed = speed;
        rollerMotor.set(percentMaxOutput * speed);
    }

    public void setChamberSpeed(double speed) {
        chamberSpeed = speed;
        chamberMotor.set(percentMaxOutput * speed * 4);
    }

}
