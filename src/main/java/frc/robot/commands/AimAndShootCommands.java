package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.Logger;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AimAndShootCommands {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    public AimAndShootCommands(SwerveDriveSubsystem swerveDriveSubsystem, VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command stoppedShootAndAimCommand() {
        return stoppedShootAndAimCommand(Optional.empty());
    }

    public Command stoppedShootAndAimCommand(Optional<Double> timeout) {
            final double angularTolerance = 0.015;
            final double velocityTolerance = 0.01;
    
            Supplier<Pose2d> getPose = () -> swerveDriveSubsystem.getPose();
    
            BooleanSupplier readyToFire = () -> (swerveDriveSubsystem.isAtDirectionCommand(angularTolerance, velocityTolerance));
    
            ProfiledPIDController controller = new ProfiledPIDController(10, 0, .5, new TrapezoidProfile.Constraints(4.5, 8));
    
            Command aimAtTag = swerveDriveSubsystem.directionCommand(() -> {
                Rotation2d output = visionSubsystem.getSpeakerAngle(getPose.get()).plus(new Rotation2d(Math.PI));
                System.out.println(output);
                Logger.log("/ShooterSubsystem/targetRotationForShooting", output.getRadians());
                return output;
            }, 
                () -> 0, () -> 0, controller, true);
    
            
    
            Command spinUpCommand = shooterSubsystem.shootCommand(() -> visionSubsystem.getSpeakerDistance(getPose.get()));
    
            Command runBeltCommand;
    
            if (timeout.isEmpty()) {
                runBeltCommand = intakeSubsystem.shootCommand();
            } else {
                runBeltCommand = intakeSubsystem.shootCommand().withTimeout(timeout.get());
            }
    
            return deadline(waitUntil(readyToFire).andThen(runBeltCommand.asProxy()), aimAtTag, spinUpCommand.asProxy());
        }

    public Command movingAimCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate) {
        final double angularTolerance = 0.015;
        final double velocityTolerance = 0.01;

        BooleanSupplier readyToFire = () -> swerveDriveSubsystem.isAtDirectionCommand(angularTolerance, velocityTolerance);

        ProfiledPIDController controller = new ProfiledPIDController(5, 0, .5, new TrapezoidProfile.Constraints(4.5, 8));

        

        Command aimAtTag = swerveDriveSubsystem.directionCommand(() -> visionSubsystem.getSpeakerAngle(swerveDriveSubsystem.getPose()).plus(new Rotation2d(Math.PI)), 
            forward, strafe, controller);

        Command spinUpCommand = shooterSubsystem.shootCommand(() -> visionSubsystem.getSpeakerDistance(swerveDriveSubsystem.getPose()));

        return parallel(aimAtTag, spinUpCommand, run(() -> {readyToFire.getAsBoolean();}));
    }

    public Command adaptiveMovingAimCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate) {
        final double angularTolerance = 0.015;
        final double velocityTolerance = 0.01;

        // This is in theory the inverse of the speed of the note
        final double forwardPredictionCoefficient = 1 / 2.0;

        BooleanSupplier readyToFire = () -> swerveDriveSubsystem.isAtDirectionCommand(angularTolerance, velocityTolerance);

        ProfiledPIDController controller = new ProfiledPIDController(5, 0, .5, new TrapezoidProfile.Constraints(4.5, 8));

        Pose2d currentPose = swerveDriveSubsystem.getPose();
        ChassisSpeeds currentSpeed = swerveDriveSubsystem.getFieldRelativeChassisSpeeds();

        Supplier<Pose2d> predictedPose = () -> {
            double distance = visionSubsystem.getSpeakerDistance(swerveDriveSubsystem.getPose());
            return new Pose2d(
                currentPose.getX() + currentSpeed.vxMetersPerSecond * forwardPredictionCoefficient * distance,
                currentPose.getY() + currentSpeed.vyMetersPerSecond * forwardPredictionCoefficient * distance,
                new Rotation2d(currentPose.getRotation().getRadians() + currentSpeed.omegaRadiansPerSecond * forwardPredictionCoefficient * distance));
        };

        Command aimAtTag = swerveDriveSubsystem.directionCommand(() -> visionSubsystem.getSpeakerAngle(predictedPose.get()).plus(new Rotation2d(Math.PI)), 
            forward, strafe, controller);

        Command spinUpCommand = shooterSubsystem.shootCommand(() -> visionSubsystem.getSpeakerDistance(predictedPose.get()));

        return parallel(aimAtTag, spinUpCommand, run(() -> {readyToFire.getAsBoolean();}));
    }

    public Command movingAimCommandAuto() {
        final double angularTolerance = 0.015;
        final double velocityTolerance = 0.01;

        BooleanSupplier readyToFire = () -> swerveDriveSubsystem.isAtDirectionCommand(angularTolerance, velocityTolerance);

        Command aimAtTag = swerveDriveSubsystem.directionCommandAuto(() -> visionSubsystem.getSpeakerAngle(swerveDriveSubsystem.getPose()).plus(new Rotation2d(Math.PI)));

        Command spinUpCommand = shooterSubsystem.shootCommand(() -> visionSubsystem.getSpeakerDistance(swerveDriveSubsystem.getPose()));

        return parallel(aimAtTag, spinUpCommand.asProxy(), run(() -> {readyToFire.getAsBoolean();}));
    }

}
