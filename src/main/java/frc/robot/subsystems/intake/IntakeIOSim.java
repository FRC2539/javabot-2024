package frc.robot.subsystems.intake;

import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;

public class IntakeIOSim implements IntakeIO{
    private double rollerSpeed = 0;
    private double beltSpeed = 0;

    private LoggedReceiver hasRollerPieceSim = Logger.tunable("/IntakeSybsystem/hasRollerPieceSim", false);
    private LoggedReceiver hasBeltPieceSim = Logger.tunable("/IntakeSybsystem/hasBeltPieceSim", false);

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerSpeed = rollerSpeed;
        inputs.beltSpeed = beltSpeed;

        inputs.beltSensor = hasBeltPieceSim.getBoolean();
        inputs.rollerSensor = hasRollerPieceSim.getBoolean();
    }

    public void setRollerSpeed(double speed) {
        this.rollerSpeed = speed;
    } 

    public void setBeltSpeed(double speed) {
        this.beltSpeed = speed;
    }
}
