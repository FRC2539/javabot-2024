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

    public Command disabledCommand() {
        return runOnce(() -> {
            setIntakeState(IntakeState.DISABLED);
        });
    }

    public Command ejectCommand() {
        return runOnce(() -> {
            setIntakeState(IntakeState.EJECTING);
        });
    }

    public Command manualDriveCommand() {
        return runOnce(() -> {
            setIntakeState(IntakeState.MANUAL_DRIVE);
        });
    }

    public Command shootCommand() {
        return runOnce(() -> {
            setIntakeState(IntakeState.SHOOTING);
        });
    }

    public Command intakeCommand() {
        Command intakeCommand = runOnce(() -> {
            setIntakeState(IntakeState.INTAKING);
        }).until(() -> getBeltSensor())
        .andThen(waitSeconds(.2))
        .until(() -> getRollerSensor());

        Trigger isIntakeFinished = new Trigger(() -> intakeCommand.isFinished());

        isIntakeFinished.onTrue(moveCommand());

        return intakeCommand;
    }

    public Command moveCommand() {
        return runOnce(() -> {
            setIntakeState(IntakeState.MOVING);
        }).until(() -> getRollerSensor()).withTimeout(5);
    }


    private void setBelt(double speed) {
        intakeIO.setBeltSpeed(speed);
    }

    private void setRoller(double speed) {
        intakeIO.setRollerSpeed(speed);
    }

    public boolean getRollerSensor() {
        return inputs.rollerSensor;
    }

    public boolean getBeltSensor() {
        return inputs.beltSensor;
    }

    private boolean hasPiece() {
        return getRollerSensor() || getBeltSensor();
    }

    public void setIntakeState(IntakeState state) {
        this.state = state;
    }

    public void logIntakeInformation() {
        Logger.log("/IntakeSubsystem/State", state.name());
        Logger.log("/IntakeSubsystem/RollerSpeed", inputs.rollerSpeed);
        Logger.log("/IntakeSubsystem/BelSpeed", inputs.beltSpeed);
        Logger.log("/IntakeSubsystem/RollerHasPiece", inputs.rollerSensor);
        Logger.log("/IntakeSubsystem/BeltHasPiece", inputs.beltSensor);

        Logger.log("/IntakeSubsystem/HasPiece", hasPiece());

        Logger.log("/IntakeSubsystem/RollerVoltage", inputs.rollerVoltage);
        Logger.log("/IntakeSubsystem/RollerCurrent", inputs.rollerCurrent);

        Logger.log("/IntakeSubsystem/BeltVoltage", inputs.beltVoltage);
        Logger.log("/IntakeSubsystem/BeltCurrent", inputs.beltCurrent);

        Logger.log("/IntakeSubsystem/RollerTemperature", inputs.rollerTemperature);
        Logger.log("/IntakeSubsystem/BeltTemperature", inputs.beltTemperature);
    }
}
