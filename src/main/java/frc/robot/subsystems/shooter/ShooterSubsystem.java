package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.lib.interpolation.InterpolatableDouble;
import frc.lib.interpolation.InterpolatingMap;

public class ShooterSubsystem extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerIO rollerIO2;
    private PneumaticsIO pneumaticsIO;

    private final InterpolatingMap<InterpolatableDouble> topRollerMap;
    private final InterpolatingMap<InterpolatableDouble> bottomRollerMap;

    private ShooterPosition shooterPositionMap(double distance) {
        if (distance < 1) {
            return ShooterPosition.FIRST_POSITION;
        }
        else if (distance >= 1 && distance < 2) {
            return ShooterPosition.SECOND_POSITION;
        }
        else if (distance >= 2 && distance < 3) {
            return ShooterPosition.THIRD_POSITION;
        }
        else {
            return ShooterPosition.FOURTH_POSITION;
        }
    }

    public ShooterSubsystem(RollerIO rollerIO, RollerIO rollerIO2, PneumaticsIO pneumaticsIO, InterpolatingMap<InterpolatableDouble> topRollerMap, InterpolatingMap<InterpolatableDouble> bottomRollerMap) {
        this.rollerIO = rollerIO;
        this.rollerIO2 = rollerIO2;
        this.pneumaticsIO = pneumaticsIO;
        
        this.topRollerMap = topRollerMap;
        this.bottomRollerMap = bottomRollerMap;
    }

    public ShooterState calculateShooterStateForDistance(double distance) {
        return new ShooterState(
            topRollerMap.getInterpolated(distance).get().value,
            bottomRollerMap.getInterpolated(distance).get().value,
            shooterPositionMap(distance)
        );
    }

    public enum ShooterPosition {
        DISABLED,
        FIRST_POSITION,
        SECOND_POSITION,
        THIRD_POSITION,
        FOURTH_POSITION
    }

    public Command shootCommand(double topRollerSpeed, double bottomRollerSpeed) {
        return run(() -> {
            rollerIO.setSpeed(topRollerSpeed);
            rollerIO2.setSpeed(bottomRollerSpeed);
        });
    }

}
