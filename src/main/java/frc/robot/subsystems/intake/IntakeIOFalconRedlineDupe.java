package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


//Dupe file for testing purposes.
public class IntakeIOFalconRedlineDupe implements IntakeIO{
    private double chamberSpeed;
    private double rollerSpeed;

    private WPI_TalonSRX chamberMotor = new WPI_TalonSRX(9);
    private WPI_TalonSRX rollerMotor = new WPI_TalonSRX(8);

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.chamberSpeed = chamberSpeed;
        inputs.rollerSpeed = rollerSpeed;

        inputs.chamberSensor = hasChamberPiece();
        inputs.rollerSensor = hasRollerPiece();
    }

    public boolean hasRollerPiece() {
        return false;
    }

    public boolean hasChamberPiece() {
        return false;
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
