package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.interpolation.InterpolatableDouble;
import frc.lib.interpolation.InterpolatingMap;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.robot.subsystems.shooter.PivotIO.PivotIOInputs;
import frc.robot.subsystems.shooter.RollerIO.RollerIOInputs;

public class ShooterSubsystem extends SubsystemBase {
    private final double shooterSpeedTolerance = 0.1;
    private final double shooterAngleTolerance = 0.01;
    
    private RollerIO topRollerIO;
    private RollerIO bottomRollerIO;
    private PivotIO pivotIO;

    private double currentDistance;

    private final InterpolatingMap<InterpolatableDouble> topRollerMap;
    private final InterpolatingMap<InterpolatableDouble> bottomRollerMap;
    private final InterpolatingMap<InterpolatableDouble> pivotAngleMap;
    
    private RollerIOInputs topRollerInputs = new RollerIOInputs();
    private RollerIOInputs bottomRollerInputs = new RollerIOInputs();
    private PivotIOInputs pivotInputs = new PivotIOInputs();

    private final ShooterState defaultState = new ShooterState(0,0,0);

    private ShooterState currentShooterState = defaultState;

    public ShooterSubsystem(RollerIO topRollerIO, RollerIO bottomRollerIO, PivotIO pivotIO, InterpolatingMap<InterpolatableDouble> topRollerMap, InterpolatingMap<InterpolatableDouble> bottomRollerMap, InterpolatingMap<InterpolatableDouble> pivotAngleMap) {
        this.topRollerIO = topRollerIO;
        this.bottomRollerIO = bottomRollerIO;
        this.pivotIO = pivotIO;
        
        this.topRollerMap = topRollerMap;
        this.bottomRollerMap = bottomRollerMap;
        this.pivotAngleMap = pivotAngleMap;

        setDefaultCommand(disabledCommand());
    }

    public ShooterState updateShooterStateForDistance(double distance) {
        return new ShooterState(
            topRollerMap.getInterpolated(distance).get().value,
            bottomRollerMap.getInterpolated(distance).get().value,
            pivotAngleMap.getInterpolated(distance).get().value
        );
    }

    public void periodic() {
        updateShooterStateForDistance(currentDistance);
        topRollerIO.updateInputs(topRollerInputs);
        bottomRollerIO.updateInputs(bottomRollerInputs);
        pivotIO.updateInputs(pivotInputs);
        logShooterInformation();

        if (currentShooterState.isVoltageBased) {
            topRollerIO.setVoltage(currentShooterState.topRollerRPM);
            bottomRollerIO.setVoltage(currentShooterState.bottomRollerRPM);
            pivotIO.setAngle(currentShooterState.pivotAngle);
        } else {
            topRollerIO.setSpeed(currentShooterState.topRollerRPM);
            bottomRollerIO.setSpeed(currentShooterState.bottomRollerRPM);
            pivotIO.setAngle(currentShooterState.pivotAngle);
        }
    }

    /** NOTE: This does not work with voltage requests as there is no "SPEED" */
    public boolean isShooterAtPosition() {
        return MathUtils.equalsWithinError(
                currentShooterState.topRollerRPM, topRollerInputs.speed, shooterSpeedTolerance) && 
            MathUtils.equalsWithinError(
                currentShooterState.bottomRollerRPM, bottomRollerInputs.speed, shooterSpeedTolerance) && 
            MathUtils.equalsWithinError(
                currentShooterState.pivotAngle, pivotInputs.currentAngle, shooterAngleTolerance);
    }

    public Command disabledCommand() {
        return run(() -> {
            currentShooterState = defaultState;
        });
    }

    public Command shootCommand(double topRollerRPM, double bottomRollerRPM, double shooterAngle) {
        return run(() -> {
            currentShooterState = new ShooterState(topRollerRPM, bottomRollerRPM, shooterAngle);
        });
    }

    public Command shootCommand(double distance) {
        return run(() -> {
            currentShooterState = updateShooterStateForDistance(distance);
        });
    }

    public Command shootCommand(ShooterState shooterState) {
        return run(() -> {
            currentShooterState = shooterState;
        });
    }

    public Command shootCommand(Supplier<ShooterState> shooterState) {
        return run(() -> {
            currentShooterState = shooterState.get();
        });
    }

    public Command shootCommand(DoubleSupplier distance) {
        return run(() -> {
            currentShooterState = updateShooterStateForDistance(distance.getAsDouble());
        });
    }

    public Command ampCommand(){
        return shootCommand(new ShooterState(0 ,0 ,0));
    }

    public void logShooterInformation() {
        Logger.log("/ShooterSubsystem/topRollerSpeedSetpoint", currentShooterState.topRollerRPM);
        Logger.log("/ShooterSubsystem/bottomRollerSpeedSetpoint", currentShooterState.bottomRollerRPM);
        Logger.log("/ShooterSubsystem/shooterPositionSetpoint", currentShooterState.pivotAngle);
        Logger.log("/ShooterSubsystem/shooterDistanceSetpoint", currentDistance);

        Logger.log("/ShooterSubsystem/topRollerSpeed", topRollerInputs.speed);
        Logger.log("/ShooterSubsystem/bottomRollerSpeed", bottomRollerInputs.speed);
        Logger.log("/ShooterSubsystem/shooterPosition", pivotInputs.currentAngle);
        Logger.log("/ShooterSubsystem/isAtAngle", pivotInputs.atTarget);

        Logger.log("/ShooterSubsystem/isAtPosition", isShooterAtPosition());
    }
}
