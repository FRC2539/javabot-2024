package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.interpolation.InterpolatableDouble;
import frc.lib.interpolation.InterpolatingMap;
import frc.lib.logging.Logger;

public class ShooterSubsystem extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerIO rollerIO2;
    private PneumaticsIO pneumaticsIO;

    private double currentDistance;

    private final InterpolatingMap<InterpolatableDouble> topRollerMap;
    private final InterpolatingMap<InterpolatableDouble> bottomRollerMap;
    private ShooterState currentShoooterState;

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
        currentDistance = distance;
        currentShoooterState = new ShooterState(
            topRollerMap.getInterpolated(distance).get().value,
            bottomRollerMap.getInterpolated(distance).get().value,
            shooterPositionMap(distance)
        );
        return currentShoooterState;
    }

    public enum ShooterPosition {
        DISABLED,
        FIRST_POSITION,
        SECOND_POSITION
    }

    public void periodic() {
        updateShooterStateForDistance(currentDistance);
    }

    public Command shootCommand(double topRollerRPM, double bottomRollerRPM) {
        return run(() -> {
            rollerIO.setSpeed(topRollerRPM);
            rollerIO2.setSpeed(bottomRollerRPM);
        });
    }

    public void logShooterInformation(ShooterState shooterState) {
        Logger.log("/ShooterSubsystem/topRollerSpeed", shooterState.topRollerRPM);
        Logger.log("/ShooterSubsystem/bottomRollerSpeed", shooterState.bottomRollerRPM);
        Logger.log("/ShooterSubsystem/shooterPosition", shooterState.shooterPosition.toString());
    }
}
