package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        rollerIO.updateInputs(roller1Inputs);
        rollerIO2.updateInputs(roller2Inputs);
        pneumaticsIO.updateInputs(pneumaticsInputs);
        logShooterInformation();


    }

    public Command shootCommand(double topRollerRPM, double bottomRollerRPM) {
        return run(() -> {
            rollerIO.setSpeed(topRollerRPM);
            rollerIO2.setSpeed(bottomRollerRPM);
        });
    }

    public void logShooterInformation() {
        Logger.log("/ShooterSubsystem/topRollerSpeed", roller1Inputs.speed);
        Logger.log("/ShooterSubsystem/bottomRollerSpeed", roller2Inputs.speed);
        Logger.log("/ShooterSubsystem/shooterPosition", new boolean[]{pneumaticsInputs.forward1, pneumaticsInputs.forward2});
    }
}
