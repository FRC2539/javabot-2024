package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.framework.motor.MotorIO;
import frc.lib.framework.sensor.DigitalSensorIO;
import frc.lib.logging.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private MotorIO rollerIO;
    private MotorIO chamberIO;
    private DigitalSensorIO rollerSensorIO;
    private DigitalSensorIO chamberSensorIO;

    private boolean noteNeedsPositioned = false;

    private boolean inIntakingProcess = false;

    private Trigger hasPieceHighDebounce;
    private Trigger hasPieceLowDebounce;

    private final String stateLog = "/IntakeSubsystem/state";

    public IntakeSubsystem(MotorIO rollerIO, MotorIO chamberIO, DigitalSensorIO rollerSensor, DigitalSensorIO chamberSensor) {
        super();
        this.rollerIO = rollerIO;
        this.chamberIO = chamberIO;
        this.rollerSensorIO = rollerSensor;
        this.chamberSensorIO = chamberSensor;

        this.hasPieceHighDebounce = new Trigger(() -> hasPieceRaw()).debounce(1, DebounceType.kFalling);
        this.hasPieceLowDebounce = new Trigger(() -> hasPieceRaw()).debounce(0.1, DebounceType.kFalling);

        setDefaultCommand(defaultCommand());
    }

    @Override
    public void periodic() {
        rollerIO.update();
        chamberIO.update();
        rollerSensorIO.update();
        chamberSensorIO.update();

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
            chamberIO.setVoltage(chamber * 12);
            rollerIO.setVoltage(roller * 12);
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
        return rollerSensorIO.getSensor();
    }

    public boolean getChamberSensor() {
        return chamberSensorIO.getSensor();
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
        Logger.log("/IntakeSubsystem/RollerSpeed", rollerIO.getVelocity());
        Logger.log("/IntakeSubsystem/BelSpeed", chamberIO.getVelocity());
        Logger.log("/IntakeSubsystem/RollerHasPiece", rollerSensorIO.getSensor());
        Logger.log("/IntakeSubsystem/BeltHasPiece", chamberSensorIO.getSensor());

        Logger.log("/IntakeSubsystem/HasPieceRaw", hasPieceRaw());
        Logger.log("/IntakeSubsystem/HasPieceSmoothed", hasPieceSmoothed());

        Logger.log("/IntakeSubsystem/RollerVoltage", rollerIO.getVoltage());
        Logger.log("/IntakeSubsystem/RollerCurrent", rollerIO.getCurrent());

        Logger.log("/IntakeSubsystem/BeltVoltage", chamberIO.getVoltage());
        Logger.log("/IntakeSubsystem/BeltCurrent", chamberIO.getVoltage());

        Logger.log("/IntakeSubsystem/RollerTemperature", rollerIO.getTemperature());
        Logger.log("/IntakeSubsystem/BeltTemperature", chamberIO.getTemperature());
    }
}
