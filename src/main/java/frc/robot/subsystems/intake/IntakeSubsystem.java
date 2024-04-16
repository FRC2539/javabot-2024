package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.logging.Logger;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO intakeIO;

    private IntakeIOInputs inputs = new IntakeIOInputs();

    private boolean noteNeedsPositioned = false;

    private boolean inIntakingProcess = false;

    private Trigger hasPieceHighDebounce;
    private Trigger hasPieceLowDebounce;

    private final String stateLog = "/IntakeSubsystem/state";

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

        logIntakeInformation();
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
        return speedCommand(-1, -.25).alongWith(runOnce(() -> Logger.log(stateLog, "DISABLED")));
    }

    private Command speedCommand(double chamber, double roller) {
        return startEnd(() -> {
            intakeIO.setChamberSpeed(chamber);
            intakeIO.setRollerSpeed(roller);
        }, () -> {
            
        });
    }

    public Command ejectCommand() {
        return speedCommand(-1, -.25).alongWith(runOnce(() -> Logger.log(stateLog, "EJECT")));
    }

    private Command reverseMoveCommand() {
        return speedCommand(-.20, -.05).alongWith(runOnce(() -> Logger.log(stateLog, "REVERSE_MOVE")))
            .beforeStarting(() -> inIntakingProcess = true).finallyDo(() -> inIntakingProcess = false);
    }

    public Command shootCommand() {
        return speedCommand(1, .25).alongWith(runOnce(() -> Logger.log(stateLog, "SHOOT")));
    }

    public Command ampCommand() {
        return speedCommand(1, .25).alongWith(runOnce(() -> Logger.log(stateLog, "AMP")));
    }

    public Command intakeCommand() {
        return speedCommand(0.85, 0.425).alongWith(runOnce(() -> Logger.log(stateLog, "INTAKE")))
                .until(() -> hasPieceRaw())
                .beforeStarting(() -> noteNeedsPositioned = true)
                .beforeStarting(() -> inIntakingProcess = true).finallyDo(() -> inIntakingProcess = false);
    }

    public Command reverseIntakeCommand() {
        return speedCommand(-0.85, -0.425).alongWith(runOnce(() -> Logger.log(stateLog, "REVERSE_INTAKE")))
                .until(() -> hasPieceRaw())
                .beforeStarting(() -> noteNeedsPositioned = true)
                .beforeStarting(() -> inIntakingProcess = true).finallyDo(() -> inIntakingProcess = false);
    }

    private Command moveCommand() {
        return speedCommand(0.25, 0.0625).alongWith(runOnce(() -> Logger.log(stateLog, "MOVE")))
            .beforeStarting(() -> inIntakingProcess = true).finallyDo(() -> inIntakingProcess = false);
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
        if (inIntakingProcess) {
            return hasPieceHighDebounce.getAsBoolean();
        } else {
            return hasPieceLowDebounce.getAsBoolean();
        }
    }

    public void logIntakeInformation() {
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
