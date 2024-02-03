package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import frc.lib.logging.Logger;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO intakeIO;

    private IntakeIOInputs inputs = new IntakeIOInputs();

    private IntakeState state = IntakeState.DISABLED;

    public IntakeSubsystem(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;

        setDefaultCommand(disabledCommand());
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
                setChamber(0);
                setRoller(0);
                break;
            case EJECTING:
                setChamber(-1);
                setRoller(-1);
                break;
            case MANUAL_DRIVE:
                setChamber(.25);
                setRoller(.25);
                break;
            case SHOOTING:
                setChamber(1);
                setRoller(0);
                break;
            case MOVING:
                setChamber(.25);
                setRoller(.25);
                break;
            case INTAKING:
                setChamber(1);
                setRoller(1);
                break;
        }

        logIntakeInformation();
    }

    public Command disabledCommand() {
        return runEnd(() -> {
            setIntakeState(IntakeState.DISABLED);
        }, () -> {});
    }

    public Command ejectCommand() {
        return runEnd(() -> {
            setIntakeState(IntakeState.EJECTING);
        }, () -> {});
    }

    public Command manualDriveCommand() {
        return runEnd(() -> {
            setIntakeState(IntakeState.MANUAL_DRIVE);
        }, () -> {});
    }

    public Command shootCommand() {
        return runEnd(() -> {
            setIntakeState(IntakeState.SHOOTING);
        }, () -> {});
    }

    public Command intakeCommand() {
        Command intakeCommand = runEnd(() -> {
            setIntakeState(IntakeState.INTAKING);
        }, () -> {}).until(() -> getChamberSensor())
        .andThen(waitSeconds(.2))
        .until(() -> getRollerSensor());

        Trigger isIntakeFinished = new Trigger(() -> intakeCommand.isScheduled());

        isIntakeFinished.onFalse(moveCommand());

        return intakeCommand;
    }

    public Command moveCommand() {
        return runEnd(() -> {
            setIntakeState(IntakeState.MOVING);
        }, () -> {}).until(() -> getRollerSensor()).withTimeout(5);
    }


    private void setChamber(double speed) {
        intakeIO.setChamberSpeed(speed);
    }

    private void setRoller(double speed) {
        intakeIO.setRollerSpeed(speed);
    }

    public boolean getRollerSensor() {
        return inputs.rollerSensor;
    }

    public boolean getChamberSensor() {
        return inputs.chamberSensor;
    }

    private boolean hasPiece() {
        return getRollerSensor() || getChamberSensor();
    }

    public void setIntakeState(IntakeState state) {
        this.state = state;
    }

    public void logIntakeInformation() {
        Logger.log("/IntakeSubsystem/State", state.name());
        Logger.log("/IntakeSubsystem/RollerSpeed", inputs.rollerSpeed);
        Logger.log("/IntakeSubsystem/BelSpeed", inputs.chamberSpeed);
        Logger.log("/IntakeSubsystem/RollerHasPiece", inputs.rollerSensor);
        Logger.log("/IntakeSubsystem/BeltHasPiece", inputs.chamberSensor);

        Logger.log("/IntakeSubsystem/HasPiece", hasPiece());

        Logger.log("/IntakeSubsystem/RollerVoltage", inputs.rollerVoltage);
        Logger.log("/IntakeSubsystem/RollerCurrent", inputs.rollerCurrent);

        Logger.log("/IntakeSubsystem/BeltVoltage", inputs.chamberVoltage);
        Logger.log("/IntakeSubsystem/BeltCurrent", inputs.chamberCurrent);

        Logger.log("/IntakeSubsystem/RollerTemperature", inputs.rollerTemperature);
        Logger.log("/IntakeSubsystem/BeltTemperature", inputs.chamberTemperature);
    }
}
