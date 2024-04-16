package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.logging.Logger;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO intakeIO;

    private IntakeIOInputs inputs = new IntakeIOInputs();

    private IntakeState state = IntakeState.DISABLED;

    private boolean noteNeedsPositioned = false;

    private BooleanSupplier hasPieceHighDebounce;
    private BooleanSupplier hasPieceLowDebounce;

    public sealed interface IntakeState {
        public default String name() {
            return toString();
        }
        ;

        public record Speed(double top, double bottom, String name) implements IntakeState {}

        public static final IntakeState DISABLED = new IntakeState.Speed(0, 0, "DISABLED");
        public static final IntakeState MOVING = new IntakeState.Speed(1.0 / 4, .25 / 4, "MOVING");
        public static final IntakeState MOVING_REVERSE = new IntakeState.Speed(-.20, -.05, "MOVING_REVERSE");
        public static final IntakeState INTAKING = new IntakeState.Speed(1.0 * .85, .50 * .85, "INTAKING");
        public static final IntakeState INTAKING_REVERSE =
                new IntakeState.Speed(-1.0 * .85, -.50 * .85, "INTAKING_REVERSE");
        public static final IntakeState SHOOTING = new IntakeState.Speed(1, .25, "SHOOTING");
        public static final IntakeState AMPING = new IntakeState.Speed(1, .25, "AMPING");
        public static final IntakeState EJECTING = new IntakeState.Speed(-1, -.25, "EJECTING");
    }

    public IntakeSubsystem(IntakeIO intakeIO) {
        super();
        this.intakeIO = intakeIO;

        this.hasPieceHighDebounce = new Trigger(() -> hasPieceRaw()).debounce(1, DebounceType.kFalling);
        this.hasPieceLowDebounce = new Trigger(() -> hasPieceRaw()).debounce(0.1, DebounceType.kFalling);

        setDefaultCommand(defaultCommand());
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(inputs);

        if (state instanceof IntakeState.Speed) {
            intakeIO.setChamberSpeed(((IntakeState.Speed) state).top);
            intakeIO.setRollerSpeed(((IntakeState.Speed) state).bottom);
        }

        logIntakeInformation();
    }

    public Command stateCommand(IntakeState state) {
        return run(() -> setState(state));
    }

    public Command stateCommand(Supplier<IntakeState> state) {
        return run(() -> setState(state.get()));
    }

    private void setState(IntakeState state) {
        this.state = state;
    }

    private Command defaultCommand() {
        return Commands.either(
                moveCommand()
                        .withTimeout(0.2)
                        .andThen(reverseMoveCommand().withTimeout(0.04))
                        .andThen(reverseMoveCommand().until(() -> hasPieceRaw()))
                        .withTimeout(2)
                        .beforeStarting(() -> noteNeedsPositioned = false),
                disabledCommand(),
                () -> noteNeedsPositioned);
    }

    private Command disabledCommand() {
        return stateCommand(IntakeState.DISABLED);
    }

    public Command ejectCommand() {
        return stateCommand(IntakeState.DISABLED);
    }

    private Command reverseMoveCommand() {
        return stateCommand(IntakeState.MOVING_REVERSE);
    }

    public Command shootCommand() {
        return stateCommand(IntakeState.SHOOTING);
    }

    public Command ampCommand() {
        return stateCommand(IntakeState.AMPING);
    }

    public Command intakeCommand() {
        return stateCommand(IntakeState.INTAKING)
                .until(() -> hasPieceRaw())
                .beforeStarting(() -> noteNeedsPositioned = true);
    }

    public Command shooterIntakeCommand() {
        return stateCommand(IntakeState.INTAKING_REVERSE)
                .until(() -> hasPieceRaw())
                .beforeStarting(() -> noteNeedsPositioned = true);
    }

    private Command moveCommand() {
        return stateCommand(IntakeState.MOVING);
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

    public void logIntakeInformation() {
        Logger.log("/IntakeSubsystem/State", state.toString());
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
