package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.function.Supplier;

public class DriveToPositionCommand extends Command {
    private static final TrapezoidProfile.Constraints driveConstraints = new TrapezoidProfile.Constraints(3, 3);
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(Math.PI * 3, Math.PI * 4);

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private Supplier<Pose2d> targetPoseSupplier;

    private final ProfiledPIDController driveController = new ProfiledPIDController(5.5, 0, 0, driveConstraints);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(6, 0, 0, omegaConstraints);

    private final SlewRateLimiter xSlewRater = new SlewRateLimiter(3);
    private final SlewRateLimiter ySlewRater = new SlewRateLimiter(3);

    private final boolean finishes;

    private boolean lockRotation = false;

    private boolean lockTranslation = false;

    private final Supplier<Pose2d> currentPoseSupplier;

    /**
     * Drives directly to the given pose on the field automatically.
     *
     * Should automatically accomidate for starting with a speed.
     *
     * @param swerveDriveSubsystem
     * @param targetPoseSupplier
     */
    public DriveToPositionCommand(SwerveDriveSubsystem swerveDriveSubsystem, Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> currentPoseSupplier, boolean finishes) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;
        this.finishes = finishes;

        this.currentPoseSupplier = currentPoseSupplier;

        driveController.setTolerance(0.02);
        omegaController.setTolerance(Units.degreesToRadians(1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem);
    }

    public DriveToPositionCommand(SwerveDriveSubsystem swerveDriveSubsystem, Supplier<Pose2d> targetPoseSupplier, boolean finishes) {
        this(swerveDriveSubsystem, targetPoseSupplier, swerveDriveSubsystem::getPose, finishes);
    }

    /**
     * Drives to the given pose on the field automatically)
     *
     * @param swerveDriveSubsystem
     * @param targetPose
     */
    public DriveToPositionCommand(SwerveDriveSubsystem swerveDriveSubsystem, Pose2d targetPose, boolean finishes) {
        this(swerveDriveSubsystem, () -> targetPose, finishes);
    }

    public DriveToPositionCommand(SwerveDriveSubsystem swerveDriveSubsystem, Pose2d targetPose) {
        this(swerveDriveSubsystem, targetPose, true);
    }

    public DriveToPositionCommand withLockRotation() {
        lockRotation = true;
        return this;
    }

    public DriveToPositionCommand withLockTranslation() {
        lockTranslation = true;
        return this;
    }

    @Override
    public void initialize() {
        var robotPose = getRobotPose();
        var robotVelocity = swerveDriveSubsystem.getFieldRelativeChassisSpeeds();

        var targetPose = targetPoseSupplier.get();

        var desiredPoseTranslation = getGoalTranslation(robotPose, targetPose);

        xSlewRater.reset(robotVelocity.vxMetersPerSecond);
        ySlewRater.reset(robotVelocity.vyMetersPerSecond);

        driveController.reset(
                desiredPoseTranslation.getNorm(),
                (robotVelocity.vxMetersPerSecond * desiredPoseTranslation.getX()
                                + robotVelocity.vyMetersPerSecond * desiredPoseTranslation.getY())
                        / desiredPoseTranslation.getNorm());

        driveController.setGoal(0);

        omegaController.reset(robotPose.getRotation().getRadians(), robotVelocity.omegaRadiansPerSecond);
    }

    private Pose2d getRobotPose() {
        return currentPoseSupplier.get();
    }

    @Override
    public void execute() {
        var robotPose = getRobotPose();

        var targetPose = targetPoseSupplier.get();

        var goalTranslation = getGoalTranslation(robotPose, targetPose);

        // Update controllers
        omegaController.setGoal(targetPose.getRotation().getRadians());

        var driveSpeed = driveController.calculate(goalTranslation.getNorm()) + driveController.getSetpoint().velocity;
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians())
                + omegaController.getSetpoint().velocity;

        if (driveController.atGoal()) driveSpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        var control = swerveDriveSubsystem.closedLoop;

        if (!lockTranslation) {
            control = control
            .withVelocityX(-driveSpeed * goalTranslation.getX() / goalTranslation.getNorm())
                .withVelocityY(-driveSpeed * goalTranslation.getY() / goalTranslation.getNorm());
        }

        if (!lockRotation) {
            control = control.withRotationalRate(omegaSpeed);
        }

        swerveDriveSubsystem.setControl(control);
    }

    @Override
    public boolean isFinished() {
        return driveController.atGoal() && omegaController.atGoal() && finishes;
    }

    public boolean atGoal() {
        return driveController.atGoal() && omegaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.setControl(swerveDriveSubsystem.stopped);
    }

    public Translation2d getGoalTranslation(Pose2d robotPose, Pose2d targetPose) {
        return targetPose.getTranslation().minus(robotPose.getTranslation());
    }
}
