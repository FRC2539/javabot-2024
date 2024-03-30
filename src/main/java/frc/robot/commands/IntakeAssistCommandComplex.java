package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.Logger;
import frc.lib.vision.PinholeModel3D;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.lights.LightsSubsystemB;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeAssistCommandComplex extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final VisionSubsystem visionSubsystem;

    DoubleSupplier strafeJoystick;
    DoubleSupplier forwardJoystick;
    DoubleSupplier rotationJoystick;

    PIDController intakingController = new PIDController(1.5, 0, 0.05);

    Pose2d lastPiecePosition = null;

    SlewRateLimiter slewwweerrr = new SlewRateLimiter(40);

    public IntakeAssistCommandComplex(
            SwerveDriveSubsystem swerveDriveSubsystem,
            VisionSubsystem visionSubsystem,
            LightsSubsystemB lights,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotation) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;
        forwardJoystick = forward;
        strafeJoystick = strafe;
        rotationJoystick = rotation;

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        slewwweerrr.reset(0);
        lastPiecePosition = null;
    }

    @Override
    public void execute() {

        double joystickX = forwardJoystick.getAsDouble();
        double joystickY = strafeJoystick.getAsDouble();
        double joystickVectorMagnitude = Math.sqrt((Math.pow(joystickX, 2)) + (Math.pow(joystickY, 2)));

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

            Logger.log("/AutoIntakeCommand/piecePose", lastPiecePosition);
        }

        Logger.log("/AutoIntakeCommand/hasSeenPiece", lastPiecePosition != null);

        if (lastPiecePosition != null) {
            Pose2d currentPose = swerveDriveSubsystem.getPose();

            Rotation2d rotation = currentPose.getRotation();

            double forwardComponent = joystickX * rotation.getCos() + joystickY * rotation.getSin();

            DoubleSupplier forwardRobotRelative = () -> forwardComponent;

            Translation2d translationToPiece =
                    lastPiecePosition.getTranslation().minus(currentPose.getTranslation());

            double strafeAmount = translationToPiece
                    .rotateBy(currentPose.getRotation().unaryMinus())
                    .getY();

            Logger.log("/AutoIntakeCommand/strafeAmount", strafeAmount);

            swerveDriveSubsystem.setControl(swerveDriveSubsystem
                    .openLoopRobotCentric
                    .withVelocityX(forwardRobotRelative.getAsDouble())
                    .withVelocityY(-intakingController.calculate(strafeAmount))
                    .withRotationalRate(rotationJoystick.getAsDouble()));
            LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.green);
        } else {
            swerveDriveSubsystem.setControl(swerveDriveSubsystem
                    .openLoop
                    .withVelocityX(joystickX)
                    .withVelocityY(joystickY)
                    .withRotationalRate(rotationJoystick.getAsDouble()));
        }
    }

    @Override
    public void end(boolean interrupted) {}
}
