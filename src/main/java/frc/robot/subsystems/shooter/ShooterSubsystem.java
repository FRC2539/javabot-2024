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
import frc.lib.framework.motor.MotorIO;
import frc.lib.interpolation.InterpolatableDouble;
import frc.lib.interpolation.InterpolatingMap;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.robot.Constants.ShooterConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.sound.midi.MidiChannel;

public class ShooterSubsystem extends SubsystemBase {
    private final double shooterSpeedTolerance = 0.02;
    private final Rotation2d shooterAngleTolerance = Rotation2d.fromDegrees(0.5);

    private double topRollerAverage;
    private double bottomRollerAverage;

    private MotorIO topRollerIO;
    private MotorIO bottomRollerIO;
    private PivotIO pivotIO;

    private double currentDistance;

    private MechanismLigament2d shooter;

    private final InterpolatingMap<InterpolatableDouble> topRollerMap;
    private final InterpolatingMap<InterpolatableDouble> bottomRollerMap;
    private final InterpolatingMap<InterpolatableDouble> pivotAngleMap;

    private static final DCMotor exampleMotor = DCMotor.getFalcon500(1).withReduction(ShooterConstants.gearRatioRoller);

    private final ShooterState defaultState = new ShooterState.VoltageVoltage(0,0,0);

    private ShooterState currentShooterState = defaultState;

    private Rotation2d pitchCorrection = Rotation2d.fromDegrees(1.6);

    private boolean isShooterAtPosition = false;

    public boolean inPositionDisableMode = false;

    public interface ShooterState {
        public interface PivotPosition {
            public Rotation2d pivot();
        }

        public interface PivotVoltage{
            public double pivot();
        }

        public interface RollerVoltage {
            public double top();
            public double bottom();
        }

        public interface RollerVelocity {
            public double top();
            public double bottom();
        }


        public record VelocityPosition(double top, double bottom, Rotation2d pivot) implements ShooterState, RollerVelocity, PivotPosition {}
        public record VelocityVoltage(double top, double bottom, double pivot) implements ShooterState, RollerVelocity, PivotVoltage {}
        public record VoltagePosition(double top, double bottom, Rotation2d pivot) implements ShooterState, RollerVoltage, PivotPosition {}
        public record VoltageVoltage(double top, double bottom, double pivot) implements ShooterState, RollerVoltage, PivotVoltage {}
    }

    public ShooterSubsystem(
            MotorIO topRollerIO,
            MotorIO bottomRollerIO,
            PivotIO pivotIO,
            InterpolatingMap<InterpolatableDouble> topRollerMap,
            InterpolatingMap<InterpolatableDouble> bottomRollerMap,
            InterpolatingMap<InterpolatableDouble> pivotAngleMap,
            MechanismLigament2d shooter) {
        super();
        this.topRollerIO = topRollerIO;
        this.bottomRollerIO = bottomRollerIO;
        this.pivotIO = pivotIO;

        this.topRollerMap = topRollerMap;
        this.bottomRollerMap = bottomRollerMap;
        this.pivotAngleMap = pivotAngleMap;

        this.shooter = shooter;

        setDefaultCommand(disabledCommand());
    }

    public ShooterState calculateShooterStateFromDistance(double distance) {
        return new ShooterState.VelocityPosition(
                topRollerMap.getInterpolated(distance).get().value,
                bottomRollerMap.getInterpolated(distance).get().value,
                Rotation2d.fromDegrees(pivotAngleMap.getInterpolated(distance).get().value)
                        .plus(getPitchCorrection()));
    }

    public void setShooterState(ShooterState shooterState) {
        currentShooterState = shooterState;
    }

    @Override
    public void periodic() {
        logShooterInformation();

        calculateShooterStateFromDistance(currentDistance);
        topRollerIO.update();
        bottomRollerIO.update();
        pivotIO.update();

        if (currentShooterState instanceof ShooterState.RollerVoltage) {
            var state = (ShooterState.RollerVoltage) currentShooterState;
            topRollerIO.setVoltage(state.top() * 12);
            bottomRollerIO.setVoltage(state.bottom() * 12);
        }
        if (currentShooterState instanceof ShooterState.RollerVelocity) {
            var state = (ShooterState.RollerVelocity) currentShooterState;
            topRollerIO.setTargetVelocity(exampleMotor.getSpeed(0, state.top() * 12));
            bottomRollerIO.setTargetVelocity(exampleMotor.getSpeed(0, state.bottom() * 12));
            Logger.log(
                    "/ShooterSubsystem/topRollerSetpointSpeed",
                    exampleMotor.getSpeed(0, state.top() * 12));
            Logger.log(
                    "/ShooterSubsystem/bottomRollerSetpointSpeed",
                    exampleMotor.getSpeed(0, state.bottom() * 12));
        }

        if (currentShooterState instanceof ShooterState.PivotVoltage) {
            var state = (ShooterState.PivotVoltage) currentShooterState;
            pivotIO.setVoltage(state.pivot());
        }
        if (currentShooterState instanceof ShooterState.PivotPosition) {
            var state = (ShooterState.PivotPosition) currentShooterState;
            pivotIO.setTargetPosition(
                    Rotation2d.fromDegrees(MathUtils.ensureRange(state.pivot().getDegrees(), 9, 60)));
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
        return runOnce(() -> pivotIO.zeroPosition(angle));
    }

    public Command updateShooterAngleCommand() {
        return runOnce(() -> pivotIO.zeroPosition(pivotIO.getPosition()));
    }

    public boolean isEncoderConnected() {
        return pivotIO.isEncoderConnected();
    }

    /** NOTE: This does not work with voltage requests as there is no "SPEED" */
    private Debouncer debouncer = new Debouncer(.1, DebounceType.kRising);

    private LinearFilter movingAverageTop = LinearFilter.movingAverage(10);
    private LinearFilter movingAverageBottom = LinearFilter.movingAverage(10);

    private boolean calculateIsShooterAtPositionSpeed(double topSpeedSus, double bottomSpeedSus, Rotation2d angle) {
        double topSpeed = exampleMotor.getSpeed(0, topSpeedSus * 12);
        double bottomSpeed = exampleMotor.getSpeed(0, bottomSpeedSus * 12);

        topRollerAverage = movingAverageTop.calculate(topRollerIO.getVelocity());
        bottomRollerAverage = movingAverageBottom.calculate(bottomRollerIO.getVelocity());

        return debouncer.calculate(MathUtils.equalsWithinError(
                                topSpeed,
                                movingAverageTop.calculate(topRollerIO.getVelocity()),
                                shooterSpeedTolerance * topSpeed)
                        && MathUtils.equalsWithinError(
                                bottomSpeed,
                                movingAverageBottom.calculate(bottomRollerIO.getVelocity()),
                                shooterSpeedTolerance * bottomSpeed)
                        && inPositionDisableMode
                || MathUtils.equalsWithinError(
                        angle.getDegrees(),
                        pivotIO.getPositionAsRotation2d().getDegrees(),
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
        return pivotIO.getPositionAsRotation2d().minus(currentShooterState.pivotAngle).plus(new Rotation2d());
    }

    public Command disabledCommand() {
        return Commands.waitSeconds(0.2).andThen(run(() -> {
            currentShooterState = defaultState;
        }));
    }

    public Command shootCommand(double topRollerRPM, double bottomRollerRPM, Rotation2d shooterAngle) {
        return run(() -> {
            currentShooterState = new ShooterState.VelocityPosition(topRollerRPM, bottomRollerRPM, shooterAngle);
        });
    }

    public Command shootCommand(double distance) {
        return run(() -> {
            currentShooterState = calculateShooterStateFromDistance(distance);
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
            currentShooterState = calculateShooterStateFromDistance(distance.getAsDouble());
        });
    }

    public Command ampCommand() {
        return shootCommand(new ShooterState.VoltageVoltage(0, 0,0));
    }

    public void logShooterInformation() {
        shooter.setAngle(180 - pivotIO.getPositionAsRotation2d().getDegrees());

        Logger.log("/ShooterSubsystem/shooterDistanceSetpoint", currentDistance);

        Logger.log("/ShooterSubsystem/topRollerSpeed", topRollerIO.getVelocity());
        Logger.log("/ShooterSubsystem/bottomRollerSpeed", bottomRollerIO.getVelocity());
        Logger.log("/ShooterSubsystem/shooterPosition", pivotIO.getPosition());
        Logger.log("/ShooterSubsystem/shooterPositionDegrees", pivotIO.getPositionAsRotation2d().getDegrees());

        Logger.log("/ShooterSubsystem/isAtPosition", isShooterAtPosition());
        Logger.log("/ShooterSubsystem/pitchCorrection", pitchCorrection.getDegrees());
    }
}
