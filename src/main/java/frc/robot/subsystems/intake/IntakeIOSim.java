package frc.robot.subsystems.intake;

import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;

public class IntakeIOSim implements IntakeIO{
    private double rollerSpeed = 0;
    private double chamberSpeed = 0;

    private LoggedReceiver hasRollerPieceSim = Logger.tunable("/IntakeSybsystem/hasIntakePieceSim", false);
    private LoggedReceiver hasChamberPieceSim = Logger.tunable("/IntakeSybsystem/hasChamberPieceSim", false);

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerSpeed = rollerSpeed;
        inputs.chamberSpeed = chamberSpeed;

        inputs.chamberSensor = hasChamberPieceSim.getBoolean();
        inputs.rollerSensor = hasRollerPieceSim.getBoolean();
    }

    public void setRollerSpeed(double speed) {
        this.rollerSpeed = speed;
    } 

    public void setChamberSpeed(double speed) {
        this.chamberSpeed = speed;
    }
}
