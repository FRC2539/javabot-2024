package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.Logger;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import java.util.function.Supplier;

public class ClimberSubsystem extends SubsystemBase {
    private ClimberIO pivotIO;

    private ClimberIOInputs climberVoltage = new ClimberIOInputs();

    private MechanismLigament2d climber;

    private ClimberState state = ClimberState.disabled;

    private final double loweredHeight = 0;
    private final double raisedHeight = 190;

    public sealed interface ClimberState {
        public default String name() {
            return toString();
        }
        ;

        public record Voltage(double voltage) implements ClimberState {}

        public record Position(double position) implements ClimberState {}

        public static final ClimberState lower = new Voltage(-12);
        public static final ClimberState raise = new Voltage(12);
        public static final ClimberState disabled = new Voltage(0);
    }

    public ClimberSubsystem(ClimberIO pivotIO, MechanismLigament2d climber) {
        super();
        this.pivotIO = pivotIO;

        this.climber = climber;

        setDefaultCommand(defaultCommand());
    }

    @Override
    public void periodic() {
        logClimberInformation();

        this.pivotIO.updateInputs(climberVoltage);

        if (state instanceof ClimberState.Voltage) {
            pivotIO.setVoltage(((ClimberState.Voltage) state).voltage);
        }

        if (state instanceof ClimberState.Position) {
            // TODO: Implement This
            throw new UnsupportedOperationException("Position control implemented");
        }
    }

    private void setState(ClimberState state) {
        this.state = state;
    }

    public Command stateCommand(ClimberState state) {
        return run(() -> setState(state));
    }

    public Command stateCommand(Supplier<ClimberState> state) {
        return run(() -> setState(state.get()));
    }

    private Command defaultCommand() {
        return disabledCommand();
    }

    public Command disabledCommand() {
        return stateCommand(ClimberState.disabled);
    }

    public Command voltageCommand(double voltage) {
        return stateCommand(new ClimberState.Voltage(voltage));
    }

    public Command zeroClimberCommand() {
        return runOnce(() -> pivotIO.setPosition(0));
    }

    public Command raiseCommand() {
        return stateCommand(ClimberState.raise).until(() -> climberVoltage.currentPosition >= raisedHeight);
    }

    public Command primeCommand() {
        return stateCommand(ClimberState.raise).until(() -> climberVoltage.currentPosition >= raisedHeight);
    }

    public Command lowerCommand() {
        return stateCommand(ClimberState.lower).until(() -> climberVoltage.currentPosition <= loweredHeight);
    }

    public void logClimberInformation() {
        climber.setLength(Units.inchesToMeters(22.5) + climberVoltage.currentPosition / 190 * Units.inchesToMeters(21));

        Logger.log("/ClimberSubsystem/position", climberVoltage.currentPosition);
        Logger.log("/ClimberSubsystem/state", state.toString());
        Logger.log("/ClimberSubsystem/voltage", climberVoltage.currentVoltage);
    }
}
