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
    private static final TrapezoidProfile.Constraints driveConstraints = new TrapezoidProfile.Constraints(0.5, 1);
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private Supplier<Pose2d> targetPoseSupplier;

    private final ProfiledPIDController driveController = new ProfiledPIDController(1, 0, 0, driveConstraints);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 0, 0, omegaConstraints);

    private final SlewRateLimiter xSlewRater = new SlewRateLimiter(3);
    private final SlewRateLimiter ySlewRater = new SlewRateLimiter(3);

    private final boolean finishes;

    /**
     * Drives directly to the given pose on the field automatically.
     *
     * Should automatically accomidate for starting with a speed.
     *
     * @param swerveDriveSubsystem
     * @param targetPoseSupplier
     */
    public DriveToPositionCommand(SwerveDriveSubsystem swerveDriveSubsystem, Supplier<Pose2d> targetPoseSupplier, boolean finishes) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;
        this.finishes = finishes;

        driveController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem);
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

    @Override
    public void initialize() {
        var robotPose = swerveDriveSubsystem.getPose();
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

    @Override
    public void execute() {
        var robotPose = swerveDriveSubsystem.getPose();

        var targetPose = targetPoseSupplier.get();

        var goalTranslation = getGoalTranslation(robotPose, targetPose);

        // Update controllers
        omegaController.setGoal(targetPose.getRotation().getRadians());

        var driveSpeed = driveController.calculate(goalTranslation.getNorm()) + driveController.getSetpoint().velocity;
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians())
                + omegaController.getSetpoint().velocity;

        if (driveController.atGoal()) driveSpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        swerveDriveSubsystem.setControl(
                swerveDriveSubsystem.closedLoop
                .withVelocityX(-driveSpeed * goalTranslation.getX() / goalTranslation.getNorm())
                .withVelocityY(-driveSpeed * goalTranslation.getY() / goalTranslation.getNorm())
                .withRotationalRate(omegaSpeed));
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
