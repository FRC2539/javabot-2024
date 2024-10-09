package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.logging.Logger;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import java.util.function.BooleanSupplier;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO intakeIO;

    private IntakeIOInputs inputs = new IntakeIOInputs();

    private IntakeState state = IntakeState.DISABLED;

    private BooleanSupplier hasPieceHighDebounce;
    private BooleanSupplier hasPieceLowDebounce;

    public double topSpeedAdjustable = 0;
    public double bottomSpeedAdjustable = 0;

    public IntakeSubsystem(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;

        this.hasPieceHighDebounce = new Trigger(() -> hasPieceRaw()  || inputs.rollerCurrent > 90).debounce(1, DebounceType.kFalling);
        this.hasPieceLowDebounce = new Trigger(() -> hasPieceRaw()  || inputs.rollerCurrent > 90).debounce(0.1, DebounceType.kFalling);

        setDefaultCommand(Commands.either(
                moveCommand()
                        .withTimeout(0.2)
                        .andThen(reverseMoveCommand().withTimeout(0.04))
                        // .andThen(reverseMoveCommand().until(() -> getRollerSensor()).withTimeout(2)),
                        .andThen(reverseMoveCommand().until(() -> hasPieceRaw()))
                        .withTimeout(2),
                // .andThen(moveCommand().withTimeout(0.0))),
                disabledCommand(),
                () -> state == IntakeState.INTAKING || state == IntakeState.INTAKING_REVERSE));
    }

    public enum IntakeState {
        DISABLED,
        MOVING,
        MOVING_REVERSE,
        INTAKING,
        INTAKING_REVERSE,
        SHOOTING,
        AMPING,
        EJECTING,
        ADJUSTABLE,
        CURLING
    }

    public void periodic() {
        intakeIO.updateInputs(inputs);

        // Roller moves 4 times as fast as chamber

        switch (state) {
            case DISABLED:
                setChamber(0);
                setRoller(0);
                break;
            case EJECTING:
                setChamber(-1);
                setRoller(-.25);
                break;
            case CURLING:
                setChamber(-1);
                setRoller(-1);
                break;
            case SHOOTING:
                setChamber(1 * 12 / 12.0);
                setRoller(.25 * 12 / 12.0);
                break;
            case AMPING:
                setChamber(1 * 9/12.0);
                setRoller(.25 * 9.0/12.0);
                break;
            case MOVING:
                setChamber(1.0 / 4);
                setRoller(.25 / 4);
                break;
            case MOVING_REVERSE:
                setChamber(-.20);
                setRoller(-.05);
                break;
            case INTAKING:
                setChamber(1.0 * .85); // .85
                setRoller(.50 * .85); // .85
                break;
            case INTAKING_REVERSE:
                setChamber(-1.0 * .85);
                setRoller(-.50 * .85);
                break;

            case ADJUSTABLE:
                setRoller(topSpeedAdjustable);
                setChamber(bottomSpeedAdjustable);
                break;
        }

        logIntakeInformation();
    }

    public Command disabledCommand() {
        return runEnd(
                () -> {
                    setIntakeState(IntakeState.DISABLED);
                },
                () -> {});
    }

    public Command ejectCommand() {
        return runEnd(
                () -> {
                    setIntakeState(IntakeState.EJECTING);
                },
                () -> {});
    }

    public Command curlCommand() {
        return runEnd(
                () -> {
                    setIntakeState(IntakeState.CURLING);
                },
                () -> {});
    }

    public Command reverseMoveCommand() {
        return runEnd(
                () -> {
                    setIntakeState(IntakeState.MOVING_REVERSE);
                },
                () -> {});
    }

    public Command shootCommand() {
        return runEnd(
                () -> {
                    setIntakeState(IntakeState.SHOOTING);
                },
                () -> {});
    }

    public Command ampCommand() {
        return runEnd(
                () -> {
                    setIntakeState(IntakeState.AMPING);
                },
                () -> {});
    }

    public Command intakeCommand() {
        Command intakeCommand = runEnd(
                        () -> {
                            setIntakeState(IntakeState.INTAKING);
                        },
                        () -> {})
                .until(() -> hasPieceRaw());

        return intakeCommand;
    }

    public Command shooterIntakeCommand() {
        return runEnd(
                        () -> {
                            setIntakeState(IntakeState.INTAKING_REVERSE);
                        },
                        () -> {})
                .until(() -> hasPieceRaw());
    }

    public Command moveCommand() {
        return runEnd(
                () -> {
                    setIntakeState(IntakeState.MOVING);
                },
                () -> {});
    }

    public Command manualMoveCommand() {
        return runEnd(
                () -> {
                    setIntakeState(IntakeState.MOVING);
                },
                () -> {});
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

    public boolean hasPieceRaw() {
        return getRollerSensor() || getChamberSensor();
    }

    public boolean hasPieceSmoothed() {
        if (state == IntakeState.INTAKING
                || state == IntakeState.MOVING
                || state == IntakeState.INTAKING_REVERSE
                || state == IntakeState.MOVING_REVERSE) {
            return hasPieceHighDebounce.getAsBoolean();
        } else {
            return hasPieceLowDebounce.getAsBoolean();
        }
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

        Logger.log("/IntakeSubsystem/HasPieceRaw", hasPieceRaw());
        Logger.log("/IntakeSubsystem/HasPieceSmoothed", hasPieceSmoothed());

        Logger.log("/IntakeSubsystem/RollerVoltage", inputs.rollerVoltage);
        Logger.log("/IntakeSubsystem/RollerCurrent", inputs.rollerCurrent);

        Logger.log("/IntakeSubsystem/BeltVoltage", inputs.chamberVoltage);
        Logger.log("/IntakeSubsystem/BeltCurrent", inputs.chamberCurrent);

        Logger.log("/IntakeSubsystem/RollerTemperature", inputs.rollerTemperature);
        Logger.log("/IntakeSubsystem/BeltTemperature", inputs.chamberTemperature);
    }
}
