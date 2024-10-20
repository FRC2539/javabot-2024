package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.lib.logging.Logger.log;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {
    // IO
    private IntakeIO intakeIO;
    private IntakeIOInputs inputs = new IntakeIOInputs();

    // Tunables
    public double topSpeedAdjustable = 0;
    public double bottomSpeedAdjustable = 0;

    // Non-command State
    private boolean needsToZero = false;
    private boolean intaking = false;
    private String stateName = "disabled";

    // Constructor(s)
    public IntakeSubsystem(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;

        setDefaultCommand(Commands.either(
                move()
                        .withTimeout(0.2)
                        .andThen(reverseMove().withTimeout(0.04))
                        // .andThen(reverseMoveCommand().until(() -> getRollerSensor()).withTimeout(2)),
                        .andThen(reverseMove().until(hasPieceRaw))
                        .withTimeout(2).andThen(() -> needsToZero = false),
                // .andThen(moveCommand().withTimeout(0.0))),
                disabled(),
                () -> needsToZero));
    }

    // Periodic
    @Override
    public void periodic() {
        intakeIO.updateInputs(inputs);
        logIntakeInformation();
    }

    // Main (Stateful) Commands
    public Command disabled() {
        return nameCommand(runIntakeAtVoltages(0,0),"disabled");
    }

    public Command eject() {
        return nameCommand(runIntakeAtVoltages(-1,-.25),"eject");
    }

    public Command reverseMove() {
        return nameCommand(runIntakeAtVoltages(-.20,-.05),"reverseMove");
    }

    public Command shoot() {
        return nameCommand(runIntakeAtVoltages(1 * 12 / 12.0,.25 * 12 / 12.0),"shoot");
    }

    public Command amp() { 
        return nameCommand(runIntakeAtVoltages(1 * 9/12.0,.25 * 9.0/12.0),"amp");
    };

    public Command intake() {
        return nameCommand(intakingCommand(runIntakeAtVoltages(1.0 * .85,.50 * .85)
                .until(hasPieceRaw).finallyDo(() -> needsToZero = true)),"intake");
    }

    public Command shooterIntake() {
        return nameCommand(intakingCommand(runIntakeAtVoltages(-1.0 * .85,-.50 * .85)
                .until(hasPieceRaw).finallyDo(() -> needsToZero = true)),"shooterIntake");
    }

    public Command move() {
        return nameCommand(intakingCommand(runIntakeAtVoltages(1.0 / 4,.25 / 4)),"move");
    }

    public Command manualMove() {
        return nameCommand(intakingCommand(runIntakeAtVoltages(1.0 / 4,.25 / 4)),"manualMove");
    }

    // Command Helpers
    public Command runIntakeAtVoltages(double chamberVoltage, double rollerVoltage) {
        return run(
            () -> {
                setChamber(chamberVoltage);
                setRoller(rollerVoltage);
            }
        );
    }

    private Command intakingCommand(Command command) {
        return command.beforeStarting(() -> intaking = true).finallyDo(() -> intaking = false);
    }

    public Command nameCommand(Command command, String name) {
        return command.beforeStarting(() -> stateName = name).withName(name);
    }

    // Setter/Getter Helpers
    private void setChamber(double speed) {
        intakeIO.setChamberSpeed(speed);
    }

    private void setRoller(double speed) {
        intakeIO.setRollerSpeed(speed);
    }

    /** Replace ALL BOOLEAN MEHTODS WITH TRIGGERS HAHAHAHAHA???? im actually kinda confused on this one */
    public final Trigger getRollerSensor = new Trigger(() -> inputs.rollerSensor);
    public final Trigger getChamberSensor = new Trigger(() -> inputs.chamberSensor);

    public final Trigger hasPieceRaw = getRollerSensor.or(getChamberSensor);


    private final Trigger hasPieceHighDebounce = hasPieceRaw.debounce(1, DebounceType.kFalling);
    private final Trigger hasPieceLowDebounce = hasPieceRaw.debounce(0.1, DebounceType.kFalling);
 
    public final Trigger hasPieceSmoothed = new Trigger(() -> {
        if (intaking) {
            return hasPieceHighDebounce.getAsBoolean();
        } else {
            return hasPieceLowDebounce.getAsBoolean();
        }
    });
    
    public void logIntakeInformation() {
        log("/IntakeSubsystem/State", stateName);
        log("/IntakeSubsystem/RollerSpeed", inputs.rollerSpeed);
        log("/IntakeSubsystem/BelSpeed", inputs.chamberSpeed);
        log("/IntakeSubsystem/RollerHasPiece", inputs.rollerSensor);
        log("/IntakeSubsystem/BeltHasPiece", inputs.chamberSensor);

        log("/IntakeSubsystem/HasPieceRaw", hasPieceRaw.getAsBoolean());
        log("/IntakeSubsystem/HasPieceSmoothed", hasPieceSmoothed.getAsBoolean());

        log("/IntakeSubsystem/RollerVoltage", inputs.rollerVoltage);
        log("/IntakeSubsystem/RollerCurrent", inputs.rollerCurrent);

        log("/IntakeSubsystem/BeltVoltage", inputs.chamberVoltage);
        log("/IntakeSubsystem/BeltCurrent", inputs.chamberCurrent);

        log("/IntakeSubsystem/RollerTemperature", inputs.rollerTemperature);
        log("/IntakeSubsystem/BeltTemperature", inputs.chamberTemperature);
    }
}
