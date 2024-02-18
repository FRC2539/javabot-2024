package frc.robot.subsystems.trap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.robot.subsystems.trap.RackIO.RackIOInputs;
import frc.robot.subsystems.trap.TrapRollerIO.RollerIOInputs;

public class TrapSubsystem extends SubsystemBase {
    private final double trapSpeedTolerance = 0.1;
    private final double trapAngleTolerance = 0.01;
    
    private TrapRollerIO topRollerIO;
    private TrapRollerIO bottomRollerIO;
    private RackIO rackIO;

    private RollerIOInputs topRollerInputs = new RollerIOInputs();
    private RollerIOInputs bottomRollerInputs = new RollerIOInputs();
    private RackIOInputs rackInputs = new RackIOInputs();

    private final TrapState defaultState = new TrapState(0,0,1.2,true);

    private TrapState currentTrapState = defaultState;

    public TrapSubsystem(TrapRollerIO topRollerIO, TrapRollerIO bottomRollerIO, RackIO rackIO) {
        this.topRollerIO = topRollerIO;
        this.bottomRollerIO = bottomRollerIO;
        this.rackIO = rackIO;


        setDefaultCommand(disabledCommand());
    }

    public void periodic() {
        logTrapInformation();

        topRollerIO.updateInputs(topRollerInputs);
        bottomRollerIO.updateInputs(bottomRollerInputs);
        rackIO.updateInputs(rackInputs);

        topRollerIO.setVoltage(currentTrapState.topVoltage);
        bottomRollerIO.setVoltage(currentTrapState.bottomVoltage);

        if (currentTrapState.isVoltageBased) {
            rackIO.setVoltage(currentTrapState.rack);
        } else {
            rackIO.setPosition(MathUtils.ensureRange(currentTrapState.rack, 0.1, 1));
        }
    }

    /** NOTE: This does not work with voltage requests as there is no "SPEED" */
    public boolean isTrapAtPosition() {
        return MathUtils.equalsWithinError(
                currentTrapState.topVoltage, topRollerInputs.speed, trapSpeedTolerance) && 
            MathUtils.equalsWithinError(
                currentTrapState.bottomVoltage, bottomRollerInputs.speed, trapSpeedTolerance) && 
            MathUtils.equalsWithinError(
                currentTrapState.rack, rackInputs.position, trapAngleTolerance);
    }

    public Command disabledCommand() {
        return run(() -> {
            currentTrapState = defaultState;
        });
    }

    public Command shootCommand(double topRollerRPM, double bottomRollerRPM, double trapAngle) {
        return run(() -> {
            currentTrapState = new TrapState(topRollerRPM, bottomRollerRPM, trapAngle);
        });
    }

    public Command shootCommand(TrapState trapState) {
        return run(() -> {
            currentTrapState = trapState;
        });
    }

    public Command shootCommand(Supplier<TrapState> trapState) {
        return run(() -> {
            currentTrapState = trapState.get();
        });
    }

    public Command ampCommand(){
        return shootCommand(new TrapState(0 ,0, 0.5));
    }

    public Command manuallyMoveRackCommand(double voltage){
        return run(() -> {
            rackIO.setVoltage(voltage);
        });
    }

    public Command runIntakeCommand(double topVoltage, double bottomVoltage) {
        return shootCommand(TrapState.fromVoltages(topVoltage, bottomVoltage, 1.2));
    }

    public void logTrapInformation() {
        Logger.log("/TrapSubsystem/topRollerSpeedSetpoint", currentTrapState.topVoltage);
        Logger.log("/TrapSubsystem/bottomRollerSpeedSetpoint", currentTrapState.bottomVoltage);
        Logger.log("/TrapSubsystem/trapPositionSetpoint", currentTrapState.rack);

        Logger.log("/TrapSubsystem/topRollerSpeed", topRollerInputs.speed);
        Logger.log("/TrapSubsystem/topRollerTemperature", topRollerInputs.motorTemperature);
        Logger.log("/TrapSubsystem/bottomRollerTemperature", topRollerInputs.motorTemperature);
        Logger.log("/TrapSubsystem/rackTemperature", topRollerInputs.motorTemperature);
        Logger.log("/TrapSubsystem/bottomRollerSpeed", bottomRollerInputs.speed);
        Logger.log("/TrapSubsystem/trapPosition", rackInputs.position);
        Logger.log("/TrapSubsystem/isAtAngle", rackInputs.atTarget);

        Logger.log("/TrapSubsystem/isAtPosition", isTrapAtPosition());
    }
}
