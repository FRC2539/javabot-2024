package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.interpolation.InterpolatableDouble;
import frc.lib.interpolation.InterpolatingMap;
import frc.lib.logging.Logger;
import frc.robot.subsystems.shooter.PneumaticsIO.PneumaticsIOInputs;
import frc.robot.subsystems.shooter.RollerIO.RollerIOInputs;

public class ShooterSubsystem extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerIO rollerIO2;
    private PneumaticsIO pneumaticsIO;

    private double currentDistance;

    private final InterpolatingMap<InterpolatableDouble> topRollerMap;
    private final InterpolatingMap<InterpolatableDouble> bottomRollerMap;
    private ShooterState currentShoooterState;
    
    private RollerIOInputs roller1Inputs = new RollerIOInputs();
    private RollerIOInputs roller2Inputs = new RollerIOInputs();
    private PneumaticsIOInputs pneumaticsInputs = new PneumaticsIOInputs();

    private ShooterPosition shooterPositionMap(double distance) {
        if (distance < 1) {
            return ShooterPosition.FIRST_POSITION;
        }
        else {
            return ShooterPosition.SECOND_POSITION;
        }
    }

    public ShooterSubsystem(RollerIO rollerIO, RollerIO rollerIO2, PneumaticsIO pneumaticsIO, InterpolatingMap<InterpolatableDouble> topRollerMap, InterpolatingMap<InterpolatableDouble> bottomRollerMap) {
        this.rollerIO = rollerIO;
        this.rollerIO2 = rollerIO2;
        this.pneumaticsIO = pneumaticsIO;
        
        this.topRollerMap = topRollerMap;
        this.bottomRollerMap = bottomRollerMap;
    }

    public ShooterState updateShooterStateForDistance(double distance) {
        return new ShooterState(
            topRollerMap.getInterpolated(distance).get().value,
            bottomRollerMap.getInterpolated(distance).get().value,
            shooterPositionMap(distance)
        );
    }

    public enum ShooterPosition {
        DISABLED(false, false),
        FIRST_POSITION(false, true),
        SECOND_POSITION(true,true);

        public boolean forward1;
        public boolean forwrard2;

        ShooterPosition(boolean forward1, boolean forward2) {
            this.forward1 = forward1;
            this.forwrard2 = forward2;
        }
    }

    public void periodic() {
        updateShooterStateForDistance(currentDistance);
        rollerIO.updateInputs(roller1Inputs);
        rollerIO2.updateInputs(roller2Inputs);
        pneumaticsIO.updateInputs(pneumaticsInputs);
        logShooterInformation();

        rollerIO.setSpeed(currentShoooterState.topRollerRPM);
        rollerIO2.setSpeed(currentShoooterState.bottomRollerRPM);
        pneumaticsIO.setPosition(currentShoooterState.shooterPosition.forward1, currentShoooterState.shooterPosition.forwrard2);
    }

    public Command shootCommand(double topRollerRPM, double bottomRollerRPM, ShooterPosition shooterPosition) {
        return runOnce(() -> {
            currentShoooterState = new ShooterState(topRollerRPM, bottomRollerRPM, shooterPosition);
        });
    }

    public Command shootCommand(double distance) {
        return runOnce(() -> {
            currentShoooterState = updateShooterStateForDistance(distance);
        });
    }

    public Command shootCommand(ShooterState shooterState) {
        return runOnce(() -> {
            currentShoooterState = shooterState;
        });
    }

    public Command shootCommand(DoubleSupplier distance) {
        return run(() -> {
            currentShoooterState = updateShooterStateForDistance(distance.getAsDouble());
        });
    }

    public void logShooterInformation() {
        Logger.log("/ShooterSubsystem/topRollerSpeedSetpoint", currentShoooterState.topRollerRPM);
        Logger.log("/ShooterSubsystem/bottomRollerSpeedSetpoint", currentShoooterState.bottomRollerRPM);
        Logger.log("/ShooterSubsystem/shooterPositionSetpoint", currentShoooterState.shooterPosition.toString());
        Logger.log("/ShooterSubsystem/shooterPositionSetpoint", currentDistance);

        Logger.log("/ShooterSubsystem/topRollerSpeed", roller1Inputs.speed);
        Logger.log("/ShooterSubsystem/bottomRollerSpeed", roller2Inputs.speed);
        Logger.log("/ShooterSubsystem/shooterPosition", new boolean[]{pneumaticsInputs.forward1, pneumaticsInputs.forward2});
    }
}
