package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.robot.subsystems.lights.LightsSubsystemB;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class AimAndSpinupCommand extends Command {
    private final double toleranceMeters = .305;
    private final double toleranceVelocity = Math.toRadians(20);
    private final double pivotToleranceMeters = Math.toRadians(2); // degrees
    private final double pivotToleranceMin = Math.toRadians(.5);
    private final double timeSeenTagMinimum = 0.25;
    private final double maxSpeedPID = 4;
    private final double rollerSpeedTolerance = 20;

    private final PIDController visionPIDController = new PIDController(4, 0, .2);
    // private final ProfiledPIDController nonVisionPIDController = new ProfiledPIDController(1, 0, .1, new
    // TrapezoidProfile.Constraints(4, 8));

    // State Tracking Variables
    private boolean hasSeenTarget = false;
    private Debouncer hasSeenTargetDebouncer = new Debouncer(timeSeenTagMinimum, DebounceType.kRising);

    // Configs
    private boolean useAutoMode;
    private boolean doSpinup;
    private double motionForwardPrediction;
    private double motionForwardCompensationBase;
    private boolean usingTarget;
    private boolean doAiming;

    // Subsystems
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private ShooterSubsystem shooterSubsystem;

    @SuppressWarnings("unused")
    private LightsSubsystemB lightsSubsystem;

    private VisionSubsystem visionSubsystem;

    // Internal Variables
    private double calculatedDistance;
    private Rotation2d calculatedRotation;
    private double goalCalculatedRotationSpeed;

    // Output Variables
    private boolean isAtAngle;
    private boolean isSpunUp;
    private boolean hasTarget;

    private DoubleSupplier forward;
    private DoubleSupplier strafe;

    @SuppressWarnings("unused")
    private DoubleSupplier rotate;

    /**
     * A general purpose Aim and Spinup command. (does not actually fire)
     * @param swerveDriveSubsystem
     * @param shooterSubsystem
     * @param lightsSubsystem
     * @param visionSubsystem
     * @param useAutoMode If true, the command will use the PathPlanner rotation override instead of commanding a direct rotation to the swerve drive.
     * @param motionForwardPrediction If non-zero, the command will predict this many seconds forward per meter from the goal.
     * @param doSpinup If this is true, the command will spinup the shooter, else, it will not.
     * @param usingTarget If this is true, the command will switch to directly using the tag from the camera when it sees it for long enough.
     */
    public AimAndSpinupCommand(
            SwerveDriveSubsystem swerveDriveSubsystem,
            ShooterSubsystem shooterSubsystem,
            LightsSubsystemB lightsSubsystem,
            VisionSubsystem visionSubsystem,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotate,
            boolean useAutoMode,
            double motionForwardPrediction,
            double motionForwardCompensationBase,
            boolean doSpinup,
            boolean doAiming,
            boolean usingTarget,
            boolean dropRequirements) {
        super();

        // Assign Configs
        this.useAutoMode = useAutoMode;
        this.doSpinup = doSpinup;
        this.motionForwardPrediction = motionForwardPrediction;
        this.motionForwardCompensationBase = motionForwardCompensationBase;
        this.usingTarget = usingTarget;
        this.doAiming = doAiming;

        // Assign Subsystems
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.lightsSubsystem = lightsSubsystem;
        this.visionSubsystem = visionSubsystem;

        // Assign Control Sticks
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;

        // PID Configs
        // nonVisionPIDController.enableContinuousInput(0, 2*Math.PI);
        visionPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Add Requirements (vision and lights are not included as vision is read only, and lights shouldnt cancel the
        // command)
        if (!dropRequirements) {
            if (doAiming && !useAutoMode) {
                addRequirements(swerveDriveSubsystem);
            }
            if (doSpinup) {
                addRequirements(shooterSubsystem);
            }
        }

        setName("AimAndSpinupCommand");
    }

    @Override
    public void initialize() {
        hasSeenTargetDebouncer.calculate(false);
        hasSeenTarget = false;
        isAtAngle = false;
        isSpunUp = false;
    }

    @Override
    public void execute() {
        calculateShootingRotationAndDistance();
        commandSwerveDrive();
        spinupShooter();
        updateLights();
        Logger.log("/ShootingCommand/hasSeenTarget", hasSeenTarget);
        Logger.log("/ShootingCommand/hasTarget", hasTarget);
        Logger.log("/ShootingCommand/isAtAngle", isAtAngle);
        Logger.log("/ShootingCommand/isSpunUp", isSpunUp);
        Logger.log("/ShootingCommand/rotation", calculatedRotation.getRadians());
        Logger.log("/ShootingCommand/distance", calculatedDistance);
    }

    private void calculateShootingRotationAndDistance() {
        Pose2d predictedPose;
        Pose2d futurePose;
        Pose2d currentPose = swerveDriveSubsystem.getPose();

        // if the motion Forward Prediction is 0, skip the step (no moving while shooting)
        if (motionForwardPrediction != 0) {
            ChassisSpeeds currentSpeed = swerveDriveSubsystem.getFieldRelativeChassisSpeeds();
            double distance = visionSubsystem.getSpeakerDistanceFromPose(swerveDriveSubsystem.getPose());
            predictedPose = new Pose2d(
                    currentPose.getX() + currentSpeed.vxMetersPerSecond * motionForwardPrediction * distance,
                    currentPose.getY() + currentSpeed.vyMetersPerSecond * motionForwardPrediction * distance,
                    new Rotation2d(currentPose.getRotation().getRadians()));
            futurePose = new Pose2d(
                    currentPose.getX() + currentSpeed.vxMetersPerSecond * (motionForwardPrediction * distance + 0.20),
                    currentPose.getY() + currentSpeed.vyMetersPerSecond * (motionForwardPrediction * distance  + 0.20),
                    new Rotation2d(currentPose.getRotation().getRadians()));
        } else {
            predictedPose = swerveDriveSubsystem.getPose();
            futurePose = predictedPose;
        }

        goalCalculatedRotationSpeed = 0;

        // Supply the default distance and rotations
        if (!hasSeenTarget) {
            calculatedRotation = visionSubsystem.getSpeakerAngleFromPose(predictedPose);
            calculatedDistance = visionSubsystem.getSpeakerDistanceFromPose(predictedPose);
            if (motionForwardPrediction != 0) {
                Rotation2d futureCalculatedRotation = visionSubsystem.getSpeakerAngleFromPose(futurePose);
                goalCalculatedRotationSpeed = futureCalculatedRotation.minus(calculatedRotation).getRadians() / 0.20;
            }
        }

        // keeps track of wether this cycle we used a target
        boolean usedATarget = false;

        // if we are using the target and see the target, fill in the info
        if (usingTarget) {
            Optional<Rotation2d> calculatedRotationOptional = visionSubsystem.getSpeakerAngleFromVision(currentPose);
            OptionalDouble calculatedDistanceOptional = visionSubsystem.getSpeakerDistanceFromVision(currentPose);
            if (calculatedRotationOptional.isPresent() && calculatedDistanceOptional.isPresent()) {
                calculatedRotation = calculatedRotationOptional.get();
                calculatedDistance = calculatedDistanceOptional.getAsDouble();
                usedATarget = true;
            }
        }

        hasTarget = usedATarget;

        // if we used a target for at least the debounce length, hasSeenTarget becomes true for the rest of the command
        hasSeenTarget |= hasSeenTargetDebouncer.calculate(usedATarget);

        // Flip the Rotation Around so we shoot the right way
        calculatedRotation = calculatedRotation.plus(Rotation2d.fromRadians(Math.PI));
    }

    private void commandSwerveDrive() {
        var currentRotation = swerveDriveSubsystem.getPose().getRotation();

        var currentTolerance = toleranceMeters / calculatedDistance;

        var rotationCorrection =
                visionPIDController.calculate(currentRotation.getRadians(), calculatedRotation.getRadians());
                

        visionPIDController.setTolerance(currentTolerance, motionForwardPrediction == 0 ? toleranceVelocity : 10);

        isAtAngle = visionPIDController.atSetpoint();

        if (doAiming) {
            // var currentVelocity = swerveDriveSubsystem.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond;

            if (useAutoMode) {
                swerveDriveSubsystem.setAutoRotationOverride(calculatedRotation);
            } else {
                swerveDriveSubsystem.setControl(swerveDriveSubsystem
                        .openLoop
                        .withVelocityX(forward.getAsDouble())
                        .withVelocityY(strafe.getAsDouble())
                        .withRotationalRate(goalCalculatedRotationSpeed + MathUtils.ensureRange(rotationCorrection, -maxSpeedPID, maxSpeedPID)));
            }
        } else {
            isAtAngle = false;
        }

        Logger.log("/ShootingCommand/rotationError", visionPIDController.getPositionError());
        Logger.log("/ShootingCommand/rotationTolerance", currentTolerance);

        Logger.log("/ShootingCommand/velocityError", visionPIDController.getVelocityError());
        Logger.log("/ShootingCommand/velocityTolerance", toleranceVelocity);
    }

    private void spinupShooter() {
        if (doSpinup) {
            shooterSubsystem.setShooterState(shooterSubsystem.calculateShooterStateFromDistance(calculatedDistance));

            double topRollerError = shooterSubsystem.topRollerError();
            double bottomRollerError = shooterSubsystem.bottomRollerError();
            double pivotAngleErrorRadians = shooterSubsystem.pivotAngleError().getRadians();
            double pivotAngleToleranceRadians = Math.max(pivotToleranceMeters / calculatedDistance, pivotToleranceMin);

            isSpunUp = (Math.abs(topRollerError) < rollerSpeedTolerance
                    && Math.abs(bottomRollerError) < rollerSpeedTolerance
                    && Math.abs(pivotAngleErrorRadians) < pivotAngleToleranceRadians);

            Logger.log("/ShootingCommand/topRollerError", topRollerError);
            Logger.log("/ShootingCommand/bottomRollerError", bottomRollerError);
            Logger.log("/ShootingCommand/pivotAngleError", pivotAngleErrorRadians);
            Logger.log("/ShootingCommand/pivotTolerance", pivotAngleToleranceRadians);
        } else {
            isSpunUp = false;
        }
    }

    private void updateLights() {
        if (!hasTarget() && usingTarget) {
            LightsSubsystemB.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystemB.red, 0.3);
        } else {
            if (isAtAngle()) {
                if (isSpunUp()) {
                    LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.blue);
                } else {
                    LightsSubsystemB.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystemB.green, 0.3);
                }
            } else {
                LightsSubsystemB.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystemB.yellow, 0.3);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (useAutoMode) {
            swerveDriveSubsystem.clearAutoRotationOverride();
        }
    }

    public boolean isAtAngle() {
        return isAtAngle;
    }

    public boolean isSpunUp() {
        return isSpunUp;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public boolean hasSeenTarget() {
        return hasSeenTarget;
    }

    public boolean isAtAngleAndSpunUp() {
        return isAtAngle() && isSpunUp();
    }

    public boolean isAtAngleAndSpunUpAndTarget() {
        return isAtAngle() && isSpunUp() && hasTarget();
    }
}
