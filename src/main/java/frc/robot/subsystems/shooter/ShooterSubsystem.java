package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.interpolation.InterpolatableDouble;
import frc.lib.interpolation.InterpolatingMap;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.PivotIO.PivotIOInputs;
import frc.robot.subsystems.shooter.RollerIO.RollerIOInputs;

public class ShooterSubsystem extends SubsystemBase {
    private final double shooterSpeedTolerance = 6;
    private final Rotation2d shooterAngleTolerance = Rotation2d.fromDegrees(0.25);

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

    private static final DCMotor exampleMotor = DCMotor.getFalcon500(1).withReduction(ShooterConstants.gearRatioRoller);

    private final ShooterState defaultState = new ShooterState(0,0, new Rotation2d(), true, true);

    private ShooterState currentShooterState = defaultState;

    private Rotation2d pitchCorrection = new Rotation2d();

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
            Rotation2d.fromDegrees(pivotAngleMap.getInterpolated(distance).get().value)
        );
    }

    public void periodic() {
        logShooterInformation();

        updateShooterStateForDistance(currentDistance);
        topRollerIO.updateInputs(topRollerInputs);
        bottomRollerIO.updateInputs(bottomRollerInputs);
        pivotIO.updateInputs(pivotInputs);

        if (currentShooterState.isVoltageBased) {
            topRollerIO.setVoltage(currentShooterState.topRollerRPM * 12);
            bottomRollerIO.setVoltage(currentShooterState.bottomRollerRPM * 12);
        } else {
            topRollerIO.setSpeed(exampleMotor.getSpeed(0, currentShooterState.topRollerRPM * 12));
            bottomRollerIO.setSpeed(exampleMotor.getSpeed(0, currentShooterState.bottomRollerRPM * 12));
            Logger.log("/ShooterSubsystem/topRollerSetpointSpeed", exampleMotor.getSpeed(0, currentShooterState.topRollerRPM * 12));
            Logger.log("/ShooterSubsystem/bottomRollerSetpointSpeed", exampleMotor.getSpeed(0, currentShooterState.bottomRollerRPM * 12));
        }

        if (currentShooterState.isAngleVoltageBased) {
            pivotIO.setVoltage(currentShooterState.pivotAngle.getRotations());
        } else {
            pivotIO.setAngle(Rotation2d.fromDegrees(MathUtils.ensureRange(currentShooterState.pivotAngle.plus(pitchCorrection).getDegrees(), 20, 55)));
        }
    }

    // this code was added using the zed text editor as a test
    public Rotation2d getPitchCorrection() {
        return pitchCorrection;
    }

    public void setPitchCorrection(Rotation2d pitchCorrection) {
        this.pitchCorrection = pitchCorrection;
    }

    public void adjustPitchCorrection(Rotation2d additionalPitchCorrection) {
        this.pitchCorrection = this.pitchCorrection.plus(additionalPitchCorrection);
    }

    public Command adjustPitchCorrectionCommand(Rotation2d additionalPitchCorrection) {
        return runOnce(() -> adjustPitchCorrection(additionalPitchCorrection));
    }

    public Command zeroShooterAngleCommand(Rotation2d angle) {
        return runOnce(() -> pivotIO.updateAngle(angle));
    }

    public Command updateShooterAngleCommand() {
        return runOnce(() -> pivotIO.updateAngle(pivotInputs.currentAngle));
    }

    /** NOTE: This does not work with voltage requests as there is no "SPEED" */
    public boolean isShooterAtPosition() {
        return MathUtils.equalsWithinError(
                currentShooterState.topRollerRPM, topRollerInputs.speed, shooterSpeedTolerance) &&
            MathUtils.equalsWithinError(
                currentShooterState.bottomRollerRPM, bottomRollerInputs.speed, shooterSpeedTolerance) &&
            MathUtils.equalsWithinError(
                currentShooterState.pivotAngle.getRadians(), pivotInputs.currentAngle.getRadians(), shooterAngleTolerance.getRadians());
    }

    public Command disabledCommand() {
        return run(() -> {
            currentShooterState = defaultState;
        });
    }

    public Command shootCommand(double topRollerRPM, double bottomRollerRPM, Rotation2d shooterAngle) {
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
        return shootCommand(new ShooterState(0 ,0 , new Rotation2d()));
    }

    public void logShooterInformation() {
        Logger.log("/ShooterSubsystem/topRollerSpeedSetpoint", currentShooterState.topRollerRPM);
        Logger.log("/ShooterSubsystem/bottomRollerSpeedSetpoint", currentShooterState.bottomRollerRPM);
        Logger.log("/ShooterSubsystem/shooterPositionSetpoint", currentShooterState.pivotAngle.getRadians());
        Logger.log("/ShooterSubsystem/shooterDistanceSetpoint", currentDistance);

        Logger.log("/ShooterSubsystem/topRollerSpeed", topRollerInputs.speed);
        Logger.log("/ShooterSubsystem/bottomRollerSpeed", bottomRollerInputs.speed);
        Logger.log("/ShooterSubsystem/shooterPosition", pivotInputs.currentAngle.getRadians());
        Logger.log("/ShooterSubsystem/shooterPositionDegrees", pivotInputs.currentAngle.getDegrees());
        Logger.log("/ShooterSubsystem/isAtAngle", pivotInputs.atTarget);

        Logger.log("/ShooterSubsystem/isAtPosition", isShooterAtPosition());
    }
}
