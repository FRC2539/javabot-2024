package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO intakeIO;

    private IntakeIOInputs inputs = new IntakeIOInputs();

    private IntakeState state = IntakeState.DISABLED;

    private Timer runningTimer = new Timer();
    private Timer stopwatchTimer = new Timer();

    public IntakeSubsystem(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    public enum IntakeState {
        DISABLED,
        MOVING,
        INTAKING,
        SHOOTING,
        EJECTING,
        MANUAL_DRIVE,
    }

    public void periodic() {
        intakeIO.updateInputs(inputs);

        switch (state) {
            case DISABLED:
                setBelt(0);
                setRoller(0);
                break;
            case EJECTING:
                setBelt(-1);
                setRoller(-1);
                break;
            case MANUAL_DRIVE:
                setBelt(.25);
                setRoller(.25);
                break;
            case SHOOTING:
                setBelt(1);
                setRoller(0);
                break;
            case MOVING:
                setBelt(.25);
                setRoller(.25);
                break;
            case INTAKING:
                setBelt(1);
                setRoller(1);
                break;
        }
    }

    private void setBelt(double speed) {
        intakeIO.setBeltSpeed(speed);
    }

    private void setRoller(double speed) {
        intakeIO.setRollerSpeed(speed);
    }

    private boolean getRollerSensor() {
        return inputs.rollerSensor;
    }

    private boolean getBeltSensor() {
        return inputs.beltSensor;
    }

    public void setIntakeState(IntakeState state) {
        this.state = state;
        runningTimer.restart();
        stopwatchTimer.stop();
        stopwatchTimer.reset();
    }
}
