package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.interpolation.InterpolatableDouble;
import frc.lib.interpolation.InterpolatingMap;
import frc.lib.logging.Logger;
import frc.robot.subsystems.shooter.PivotIO.PivotIOInputs;
import frc.robot.subsystems.shooter.RollerIO.RollerIOInputs;

public class ShooterSubsystem extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerIO rollerIO2;
    private PivotIO pivotIO;

    private double currentDistance;

    private final InterpolatingMap<InterpolatableDouble> topRollerMap;
    private final InterpolatingMap<InterpolatableDouble> bottomRollerMap;
    private final InterpolatingMap<InterpolatableDouble> shooterAngleMap;
    
    private RollerIOInputs roller1Inputs = new RollerIOInputs();
    private RollerIOInputs roller2Inputs = new RollerIOInputs();
    private PivotIOInputs pivotInputs = new PivotIOInputs();

    private ShooterState currentShooterState;

    public ShooterSubsystem(RollerIO rollerIO, RollerIO rollerIO2, PivotIO pneumaticsIO, InterpolatingMap<InterpolatableDouble> topRollerMap, InterpolatingMap<InterpolatableDouble> bottomRollerMap, InterpolatingMap<InterpolatableDouble> shooterAngleMap) {
        this.rollerIO = rollerIO;
        this.rollerIO2 = rollerIO2;
        this.pivotIO = pneumaticsIO;
        
        this.topRollerMap = topRollerMap;
        this.bottomRollerMap = bottomRollerMap;
        this.shooterAngleMap = shooterAngleMap;

        setDefaultCommand(disabledCommand());
    }

    public ShooterState updateShooterStateForDistance(double distance) {
        return new ShooterState(
            topRollerMap.getInterpolated(distance).get().value,
            bottomRollerMap.getInterpolated(distance).get().value,
            shooterAngleMap.getInterpolated(distance).get().value
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
        pivotIO.updateInputs(pivotInputs);
        logShooterInformation();

        rollerIO.setSpeed(currentShooterState.topRollerRPM);
        rollerIO2.setSpeed(currentShooterState.bottomRollerRPM);
        pivotIO.setAngle(currentShooterState.pivotAngle);
    }

    public Command disabledCommand() {
        return runOnce(() -> {
            currentShooterState = new ShooterState(0,0,0);
        });
    }

    public Command shootCommand(double topRollerRPM, double bottomRollerRPM, double shooterAngle) {
        return runOnce(() -> {
            currentShooterState = new ShooterState(topRollerRPM, bottomRollerRPM, shooterAngle);
        });
    }

    public Command shootCommand(double distance) {
        return runOnce(() -> {
            currentShooterState = updateShooterStateForDistance(distance);
        });
    }

    public Command shootCommand(ShooterState shooterState) {
        return runOnce(() -> {
            currentShooterState = shooterState;
        });
    }

    public Command shootCommand(DoubleSupplier distance) {
        return run(() -> {
            currentShooterState = updateShooterStateForDistance(distance.getAsDouble());
        });
    }

    public void logShooterInformation() {
        Logger.log("/ShooterSubsystem/topRollerSpeedSetpoint", currentShooterState.topRollerRPM);
        Logger.log("/ShooterSubsystem/bottomRollerSpeedSetpoint", currentShooterState.bottomRollerRPM);
        Logger.log("/ShooterSubsystem/shooterPositionSetpoint", currentShooterState.pivotAngle);
        Logger.log("/ShooterSubsystem/shooterDistanceSetpoint", currentDistance);

        Logger.log("/ShooterSubsystem/topRollerSpeed", roller1Inputs.speed);
        Logger.log("/ShooterSubsystem/bottomRollerSpeed", roller2Inputs.speed);
        Logger.log("/ShooterSubsystem/shooterPosition", pivotInputs.currentAngle);
        Logger.log("/ShooterSubsystem/isAtAngle", pivotInputs.atTarget);
    }
}
