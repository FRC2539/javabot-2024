package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interpolation.InterpolatableDouble;
import frc.lib.interpolation.InterpolatingMap;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.PivotIO.PivotIOInputs;
import frc.robot.subsystems.shooter.RollerIO.RollerIOInputs;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {
    public final static ShooterState podiumShot = new ShooterState(.60, .60, Rotation2d.fromDegrees(37.75));
    public final static ShooterState subwooferShot = new ShooterState(.6,.6,Rotation2d.fromDegrees(62));
    public final static ShooterState ampShot = new ShooterState(.16,.22,Rotation2d.fromDegrees(58));//new ShooterState(.1, .25, Rotation2d.fromDegrees(60));
    public final static ShooterState groundFeedShot = new ShooterState(.7, .4, Rotation2d.fromDegrees(9));
    public final static ShooterState airFeed = new ShooterState(.50,
    .45,
    Rotation2d.fromDegrees(55));

    private final double shooterSpeedTolerance = 0.02;
    private final Rotation2d shooterAngleTolerance = Rotation2d.fromDegrees(0.5);

    private double topRollerAverage;
    private double bottomRollerAverage;

    private RollerIO topRollerIO;
    private RollerIO bottomRollerIO;
    private PivotIO pivotIO;

    private double currentDistance;

    private MechanismLigament2d shooter;

    private final InterpolatingMap<InterpolatableDouble> topRollerMap;
    private final InterpolatingMap<InterpolatableDouble> bottomRollerMap;
    private final InterpolatingMap<InterpolatableDouble> pivotAngleMap;
    private RollerIOInputs topRollerInputs = new RollerIOInputs();
    private RollerIOInputs bottomRollerInputs = new RollerIOInputs();
    private PivotIOInputs pivotInputs = new PivotIOInputs();

    private static final DCMotor exampleMotor = DCMotor.getFalcon500(1).withReduction(ShooterConstants.gearRatioRoller);

    public static final ShooterState defaultStateHolding = new ShooterState(
            0, 0, Rotation2d.fromDegrees(0), true, true); // new ShooterState(0,0, Rotation2d.fromDegrees(58), true, false);s

    public static final ShooterState defaultState = new ShooterState(
            0, 0, Rotation2d.fromDegrees(56.5), true, false); // new ShooterState(0,0, Rotation2d.fromDegrees(58), true, false);

    public static final ShooterState defaultZeroState = new ShooterState(
            0, 0, Rotation2d.fromDegrees(62), true, false);

    private ShooterState currentShooterState = defaultState;

    private Rotation2d pitchCorrection = Rotation2d.fromDegrees(.35);

    private boolean isShooterAtPosition = false;

    public boolean inPositionDisableMode = false;

    private boolean hasZeroedYet = false;

    public ShooterSubsystem(
            RollerIO topRollerIO,
            RollerIO bottomRollerIO,
            PivotIO pivotIO,
            InterpolatingMap<InterpolatableDouble> topRollerMap,
            InterpolatingMap<InterpolatableDouble> bottomRollerMap,
            InterpolatingMap<InterpolatableDouble> pivotAngleMap,
            MechanismLigament2d shooter) {
        this.topRollerIO = topRollerIO;
        this.bottomRollerIO = bottomRollerIO;
        this.pivotIO = pivotIO;

        this.topRollerMap = topRollerMap;
        this.bottomRollerMap = bottomRollerMap;
        this.pivotAngleMap = pivotAngleMap;

        this.shooter = shooter;

        setDefaultCommand(disabledCommand());
    }

    public ShooterState updateShooterStateForDistance(double distance) {
        return new ShooterState(
                topRollerMap.getInterpolated(distance).get().value,
                bottomRollerMap.getInterpolated(distance).get().value,
                Rotation2d.fromDegrees(pivotAngleMap.getInterpolated(distance).get().value)
                        .plus(getPitchCorrection()));
    }

    public void setShooterState(ShooterState shooterState) {
        currentShooterState = shooterState;
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
            Logger.log(
                    "/ShooterSubsystem/topRollerSetpointSpeed",
                    exampleMotor.getSpeed(0, currentShooterState.topRollerRPM * 12));
            Logger.log(
                    "/ShooterSubsystem/bottomRollerSetpointSpeed",
                    exampleMotor.getSpeed(0, currentShooterState.bottomRollerRPM * 12));
        }

        if (currentShooterState.isAngleVoltageBased) {
            pivotIO.setVoltage(currentShooterState.pivotAngle.getRotations());
        } else if (!inPositionDisableMode) {
            pivotIO.setAngle(
                    Rotation2d.fromDegrees(MathUtils.ensureRange(currentShooterState.pivotAngle.getDegrees(), 9, 70)));
        }

        isShooterAtPosition = calculateIsShooterAtPosition();
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

    public Command reengShooterAngleCommand() {
        return runOnce(() -> pivotIO.updateAngle(pivotIO.getGripperEncoderAngle()));
    }

    public boolean isEncoderConnected() {
        return pivotInputs.isEncoderConnected;
    }

    /** NOTE: This does not work with voltage requests as there is no "SPEED" */
    private Debouncer debouncer = new Debouncer(.1, DebounceType.kRising);

    private LinearFilter movingAverageTop = LinearFilter.movingAverage(10);
    private LinearFilter movingAverageBottom = LinearFilter.movingAverage(10);

    private boolean calculateIsShooterAtPosition() {
        double topSpeed = exampleMotor.getSpeed(0, currentShooterState.topRollerRPM * 12);
        double bottomSpeed = exampleMotor.getSpeed(0, currentShooterState.bottomRollerRPM * 12);

        topRollerAverage = movingAverageTop.calculate(topRollerInputs.speed);
        bottomRollerAverage = movingAverageBottom.calculate(bottomRollerInputs.speed);

        return debouncer.calculate(MathUtils.equalsWithinError(
                                topSpeed,
                                movingAverageTop.calculate(topRollerInputs.speed),
                                shooterSpeedTolerance * topSpeed)
                        && MathUtils.equalsWithinError(
                                bottomSpeed,
                                movingAverageBottom.calculate(bottomRollerInputs.speed),
                                shooterSpeedTolerance * bottomSpeed)
                        && inPositionDisableMode
                || MathUtils.equalsWithinError(
                        currentShooterState.pivotAngle.getDegrees(),
                        pivotInputs.currentAngle.getDegrees(),
                        shooterAngleTolerance.getDegrees()));
    }

    public boolean isShooterAtPosition() {
        return isShooterAtPosition;
    }

    public double topRollerError() {
        return topRollerAverage - exampleMotor.getSpeed(0, currentShooterState.topRollerRPM * 12);
    }

    public double bottomRollerError() {
        return bottomRollerAverage - exampleMotor.getSpeed(0, currentShooterState.bottomRollerRPM * 12);
    }

    public Rotation2d pivotAngleError() {
        return pivotInputs.currentAngle.minus(currentShooterState.pivotAngle).plus(new Rotation2d());
    }

    public Command disabledCommand() {
        // return Commands.waitSeconds(0.2).andThen(run(() -> {
        //     currentShooterState = defaultZeroState;
        //     if (59 > pivotIO.getGripperEncoderAngle().getDegrees() || DriverStation.isDisabled()) {
        //         pivotIO.updateAngle(pivotIO.getGripperEncoderAngle());
        //     }
        // }).withTimeout(0.5).andThen(run(() -> {
        //     currentShooterState = defaultState;
        //     pivotIO.updateAngle(Rotation2d.fromDegrees(61.5));
        // }))).ignoringDisable(inPositionDisableMode);
        return Commands.either(Commands.waitSeconds(0.2).andThen(
            run (
                () -> {
                    currentShooterState = defaultZeroState;
                }
            ).withTimeout(0.8)
        ).andThen(run (
            () -> {
                currentShooterState = defaultState;
            }
        ).beforeStarting(() -> pivotIO.updateAngle(Rotation2d.fromDegrees(61.5)))),
        run(() -> {
            currentShooterState = defaultState;
            pivotIO.updateAngle(pivotIO.getGripperEncoderAngle());
            }).until((
                () -> 1 > Math.abs(defaultState.pivotAngle.getDegrees() - pivotIO.getGripperEncoderAngle().getDegrees())
                )).andThen(runOnce(() -> hasZeroedYet = true)),
        () -> hasZeroedYet);
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

    public Command ampCommand() {
        return shootCommand(new ShooterState(0, 0, new Rotation2d()));
    }

    public void logShooterInformation() {
        shooter.setAngle(180 - pivotInputs.currentAngle.getDegrees());

        Logger.log("/ShooterSubsystem/topRollerSpeedSetpoint", currentShooterState.topRollerRPM);
        Logger.log("/ShooterSubsystem/bottomRollerSpeedSetpoint", currentShooterState.bottomRollerRPM);
        Logger.log("/ShooterSubsystem/shooterPositionSetpoint", currentShooterState.pivotAngle.getDegrees());
        Logger.log("/ShooterSubsystem/shooterDistanceSetpoint", currentDistance);

        Logger.log("/ShooterSubsystem/topRollerSpeed", topRollerInputs.speed);
        Logger.log("/ShooterSubsystem/bottomRollerSpeed", bottomRollerInputs.speed);
        Logger.log("/ShooterSubsystem/shooterPosition", pivotInputs.currentAngle.getRadians());
        Logger.log("/ShooterSubsystem/shooterPositionDegrees", pivotInputs.currentAngle.getDegrees());
        Logger.log("/ShooterSubsystem/isAtAngle", pivotInputs.atTarget);

        Logger.log("/ShooterSubsystem/isAtPosition", isShooterAtPosition());
        Logger.log("/ShooterSubsystem/pitchCorrection", pitchCorrection.getDegrees());
    }
}
