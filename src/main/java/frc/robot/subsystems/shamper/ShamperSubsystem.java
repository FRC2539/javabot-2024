package frc.robot.subsystems.shamper;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.robot.subsystems.shamper.ShamperIO.ShamperIOInputs;
import java.util.function.Supplier;

public class ShamperSubsystem extends SubsystemBase {
    private final double shamperSpeedTolerance = 0.1;
    private final double shamperAngleTolerance = 0.01;

    public final double holdingVoltage = 0.5;

    private ShamperIO shamperIO;

    private ShamperIOInputs shamperInputs = new ShamperIOInputs();

    private final double defaultState = 0;
    private final double defaultStateHolding = 0;

    private double currentShamperState = defaultState;

    private MechanismLigament2d shamper;

    public ShamperSubsystem(ShamperIO shamperIO, MechanismLigament2d shamper) {
        this.shamperIO = shamperIO;

        this.shamper = shamper;

        setDefaultCommand(disabledCommand());
    }

    public void periodic() {
        logShamperInformation();

        shamperIO.updateInputs(shamperInputs);

        // if (false) {
        //     shamperIO.setVoltage(currentShamperState);
        // } else {
        //     shamperIO.setPosition(MathUtils.ensureRange(currentShamperState, 0.0, 1.0));
        // }
    }


    /** NOTE: This does not work with voltage requests as there is no "SPEED" */
    public boolean isShamperAtPosition() {
        return MathUtils.equalsWithinError(currentShamperState, shamperInputs.position, shamperAngleTolerance);
    }

    public Command disabledCommand() {
        return retractShamperCommand();
        // run(() -> {
        //     if (shamperInputs.position < 1) {
        //         shamperIO.setVoltage(-1);
        //     } else {
        //         shamperIO.setVoltage(-1);
        //     }
        // });
    }

    public Command trapStateCommand(double trapAngle) {
        return run(() -> {
            currentShamperState = trapAngle;
        });
    }

    public Command trapStateCommand(Supplier<Double> trapState) {
        return run(() -> {
            currentShamperState = trapState.get();
        });
    }

    public Command manuallyMoveShamperCommand(double voltage) {
        return run(() -> {
            shamperIO.setVoltage(voltage);
        });
    }

    public Command extendShamperCommand() {
        return run(() -> {
            shamperIO.setPosition(3.690);
        });
    }

    public Command retractShamperCommand() {
        return run(() -> {
            shamperIO.setPosition(0);
        });
    }

    public Command zeroShamperPositionCommand() {
        return runOnce(() -> {
            shamperIO.zeroPosition();
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

    public void logShamperInformation() {
        shamper.setLength(Units.inchesToMeters(22.5) + shamperInputs.position / 34 * Units.inchesToMeters(19));

        // Logger.log("/TrapSubsystem/topRollerSpeedSetpoint", currentTrapState.topVoltage);
        // Logger.log("/TrapSubsystem/bottomRollerSpeedSetpoint", currentTrapState.bottomVoltage);
        Logger.log("/ShamperSubsystem/shamperPositionSetpoint", currentShamperState);
        // Logger.log("/TrapSubsystem/trapIsVoltageSetpoint", currentTrapState.isVoltageBased);

        // Logger.log("/TrapSubsystem/topRollerSpeed", topRollerInputs.speed);
        // Logger.log("/TrapSubsystem/topRollerTemperature", topRollerInputs.motorTemperature);
        // Logger.log("/TrapSubsystem/bottomRollerTemperature", bottomRollerInputs.motorTemperature);
        // Logger.log("/TrapSubsystem/topRollerCurrent", topRollerInputs.current);
        // Logger.log("/TrapSubsystem/bottomRollerCurrent", bottomRoller+Inputs.current);
        Logger.log("/ShamperSubsystem/shamperCurrent", shamperInputs.current);
        Logger.log("/ShamperSubsystem/shamperTemperature", shamperInputs.temperature);
        // Logger.log("/TrapSubsystem/bottomRollerSpeed", bottomRollerInputs.speed);
        Logger.log("/ShamperSubsystem/shamperPosition", shamperInputs.position);
        Logger.log("/ShamperSubsystem/isAtAngle", shamperInputs.atTarget);

        Logger.log("/ShamperSubsystem/isAtPosition", isShamperAtPosition());
    }
}