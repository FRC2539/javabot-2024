package frc.robot.subsystems.shamper;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.robot.subsystems.shamper.RackIO.RackIOInputs;
import java.util.function.Supplier;

public class TrapSubsystem extends SubsystemBase {
    private final double trapSpeedTolerance = 0.1;
    private final double trapAngleTolerance = 0.01;

    public final double holdingVoltage = 0.5;

    private RackIO rackIO;

    private RackIOInputs rackInputs = new RackIOInputs();

    private final double defaultState = 0;
    private final double defaultStateHolding = 0;

    private double currentTrapState = defaultState;

    private MechanismLigament2d trap;

    public TrapSubsystem(RackIO rackIO, MechanismLigament2d trap) {
        this.rackIO = rackIO;

        this.trap = trap;

        setDefaultCommand(disabledCommand());
    }

    public void periodic() {
        logTrapInformation();

        rackIO.updateInputs(rackInputs);

        if (false) {
            rackIO.setVoltage(currentTrapState);
        } else {
            rackIO.setPosition(MathUtils.ensureRange(currentTrapState, 0.0, 1.0));
        }
    }


    /** NOTE: This does not work with voltage requests as there is no "SPEED" */
    public boolean isTrapAtPosition() {
        return MathUtils.equalsWithinError(currentTrapState, rackInputs.position, trapAngleTolerance);
    }

    public Command disabledCommand() {
        return run(() -> {
            if (rackInputs.position < 10) {
                currentTrapState = defaultState;
            } else {
                currentTrapState = defaultStateHolding;
            }
        });
    }

    public Command trapStateCommand(double trapAngle) {
        return run(() -> {
            currentTrapState = trapAngle;
        });
    }

    public Command trapStateCommand(Supplier<Double> trapState) {
        return run(() -> {
            currentTrapState = trapState.get();
        });
    }

    public Command manuallyMoveRackCommand(double voltage) {
        return run(() -> {
            rackIO.setVoltage(voltage);
        });
    }

    public Command zeroRackPositionCommand() {
        return runOnce(() -> {
            rackIO.zeroPosition();
        });
    }

    // public Command bringRackDownCommand() {
    //     return trapStateCommand(0, 0, 2)
    //             .until(() -> rackInputs.position < 3)
    //             .andThen(waitSeconds(0.2))
    //             .andThen(trapStateCommand(0, 0, 0));
    // }

    // public Command runIntakeCommand(double topVoltage, double bottomVoltage) {
    //     return trapStateCommand(TrapState.fromVoltages(topVoltage, bottomVoltage, holdingVoltage));
    // }

    public void logTrapInformation() {
        trap.setLength(Units.inchesToMeters(22.5) + rackInputs.position / 34 * Units.inchesToMeters(19));

        // Logger.log("/TrapSubsystem/topRollerSpeedSetpoint", currentTrapState.topVoltage);
        // Logger.log("/TrapSubsystem/bottomRollerSpeedSetpoint", currentTrapState.bottomVoltage);
        Logger.log("/TrapSubsystem/trapPositionSetpoint", currentTrapState);
        // Logger.log("/TrapSubsystem/trapIsVoltageSetpoint", currentTrapState.isVoltageBased);

        // Logger.log("/TrapSubsystem/topRollerSpeed", topRollerInputs.speed);
        // Logger.log("/TrapSubsystem/topRollerTemperature", topRollerInputs.motorTemperature);
        // Logger.log("/TrapSubsystem/bottomRollerTemperature", bottomRollerInputs.motorTemperature);
        // Logger.log("/TrapSubsystem/topRollerCurrent", topRollerInputs.current);
        // Logger.log("/TrapSubsystem/bottomRollerCurrent", bottomRollerInputs.current);
        Logger.log("/TrapSubsystem/rackCurrent", rackInputs.current);
        Logger.log("/TrapSubsystem/rackTemperature", rackInputs.temperature);
        // Logger.log("/TrapSubsystem/bottomRollerSpeed", bottomRollerInputs.speed);
        Logger.log("/TrapSubsystem/trapPosition", rackInputs.position);
        Logger.log("/TrapSubsystem/isAtAngle", rackInputs.atTarget);

        Logger.log("/TrapSubsystem/isAtPosition", isTrapAtPosition());
    }
}
