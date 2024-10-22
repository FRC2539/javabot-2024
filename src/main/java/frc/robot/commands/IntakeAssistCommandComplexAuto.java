package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.Logger;
import frc.lib.vision.PinholeModel3D;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class IntakeAssistCommandComplexAuto extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private OptionalDouble strafeingAmount = OptionalDouble.empty();

    PIDController intakingController = new PIDController(4, 0, 0.00);

    Pose2d lastPiecePosition = null;

    SlewRateLimiter slewwweerrr = new SlewRateLimiter(40);

    public IntakeAssistCommandComplexAuto(
            SwerveDriveSubsystem swerveDriveSubsystem,
            VisionSubsystem visionSubsystem,
            LightsSubsystem lights) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;
        strafeingAmount = OptionalDouble.empty();
    }

    @Override
    public void initialize() {
        slewwweerrr.reset(0);
        lastPiecePosition = null;
        strafeingAmount = OptionalDouble.empty();
    }

    @Override
    public void execute() {

        if (visionSubsystem.getDetectorInfo().isPresent()) {
            double pieceAngle = visionSubsystem.getDetectorInfo().get().ty();
            double pieceHeight = visionSubsystem.getDetectorInfo().get().tx();
            Translation2d lastPieceTranslation = PinholeModel3D.getTranslationToTarget(
                    new Translation3d(
                            1,
                            Math.tan(Math.toRadians(
                                    -visionSubsystem.getDetectorInfo().get().tx())),
                            Math.tan(Math.toRadians(
                                    visionSubsystem.getDetectorInfo().get().ty()))),
                    VisionConstants.limelightRobotToCamera,
                    0);
            Pose2d poseAtTime = swerveDriveSubsystem.getPoseAtTimestamp(
                    visionSubsystem.getDetectorInfo().get().timestamp());

            Pose2d newPiecePose = poseAtTime.plus(new Transform2d(lastPieceTranslation, new Rotation2d()));

            lastPiecePosition = newPiecePose;

            Logger.log("/AutoIntakeCommand/piecePose", new Pose3d(lastPiecePosition), true);
        }

        Logger.log("/AutoIntakeCommand/hasSeenPiece", lastPiecePosition != null);

        if (lastPiecePosition != null) {
            Pose2d currentPose = swerveDriveSubsystem.getPose();

            Rotation2d rotation = currentPose.getRotation();

            Translation2d translationToPiece =
                    lastPiecePosition.getTranslation().minus(currentPose.getTranslation());

            double strafeAmount = translationToPiece
                    .rotateBy(currentPose.getRotation().unaryMinus())
                    .getY();

            Logger.log("/AutoIntakeCommand/strafeAmount", strafeAmount);

            strafeingAmount = OptionalDouble.of(-intakingController.calculate(strafeAmount));
            LightsSubsystem.LEDSegment.MainStrip.setColor(LightsSubsystem.green);
        }
    }

    public double transformStrafe(double strafe) {
        if (strafeingAmount.isEmpty()) return strafe;
        else return strafeingAmount.getAsDouble();
    }

    @Override
    public void end(boolean interrupted) {
        strafeingAmount = OptionalDouble.empty();
    }
}
