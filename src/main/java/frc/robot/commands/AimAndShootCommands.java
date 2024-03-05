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
import frc.robot.subsystems.lights.LightsSubsystemB;
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

    public Command stoppedShootAndAimCommand(LightsSubsystemB lightsSubsystem) {
        return stoppedShootAndAimCommand(Optional.empty(), lightsSubsystem);
    }

    public Command stoppedShootAndAimCommand(Optional<Double> timeout, LightsSubsystemB lightsSubsystem) {
            final double angularTolerance = 0.120;
            final double velocityTolerance = 0.04;
    
            Supplier<Pose2d> getPose = () -> swerveDriveSubsystem.getPose();
    
            BooleanSupplier readyToFire = () -> (swerveDriveSubsystem.isAtDirectionCommand(angularTolerance, velocityTolerance) && shooterSubsystem.isShooterAtPosition());
    
            ProfiledPIDController controller = new ProfiledPIDController(3, 0, .5, new TrapezoidProfile.Constraints(4.5, 8));
    
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

            Command lightsCommand = run(() -> {
                if (readyToFire.getAsBoolean()) {
                    LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.green);
                } else {
                    LightsSubsystemB.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystemB.yellow, 0.3);
                }
            }, lightsSubsystem).asProxy();
    
            return deadline(waitUntil(readyToFire).andThen(runBeltCommand.asProxy()), aimAtTag, spinUpCommand.asProxy(), lightsCommand);
        }

    public Command movingAimCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate, LightsSubsystemB lightsSubsystem) {
        final double angularTolerance = 0.1;
        final double velocityTolerance = 0.04;

        BooleanSupplier readyToFire = () -> swerveDriveSubsystem.isAtDirectionCommand(angularTolerance, velocityTolerance);

        ProfiledPIDController controller = new ProfiledPIDController(5, 0, .5, new TrapezoidProfile.Constraints(4.5, 8));

        Command aimAtTag = swerveDriveSubsystem.directionCommand(() -> visionSubsystem.getSpeakerAngle(swerveDriveSubsystem.getPose()).plus(new Rotation2d(Math.PI)), 
            forward, strafe, controller);

        Command spinUpCommand = shooterSubsystem.shootCommand(() -> visionSubsystem.getSpeakerDistance(swerveDriveSubsystem.getPose()));

        Command lightsCommand = run(() -> {
                if (visionSubsystem.hasTag == false) {
                    LightsSubsystemB.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystemB.red, 0.3);
                } else {
                    if (readyToFire.getAsBoolean()) {
                        LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.green);
                    } else {
                        LightsSubsystemB.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystemB.yellow, 0.3);
                    }
                }
            }, lightsSubsystem).asProxy();

        return parallel(aimAtTag, spinUpCommand, lightsCommand, run(() -> {readyToFire.getAsBoolean();}));
    }

    public Command turnToTagCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate, LightsSubsystemB lightsSubsystem) {
        final double angularTolerance = 0.06;
        final double velocityTolerance = 0.02;

        BooleanSupplier readyToFire = () -> swerveDriveSubsystem.isAtDirectionCommand(angularTolerance, velocityTolerance);

        ProfiledPIDController controller = new ProfiledPIDController(5, 0, .5, new TrapezoidProfile.Constraints(4.5, 8));

        Command aimAtTag = swerveDriveSubsystem.directionCommand(() -> visionSubsystem.getSpeakerAngle(swerveDriveSubsystem.getPose()).plus(new Rotation2d(Math.PI)), 
            forward, strafe, controller);

        Command lightsCommand = run(() -> {
                if (readyToFire.getAsBoolean()) {
                    LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.green);
                } else {
                    LightsSubsystemB.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystemB.yellow, 0.3);
                }
            }, lightsSubsystem).asProxy();

        return parallel(aimAtTag, lightsCommand, run(() -> {readyToFire.getAsBoolean();}));
    }

    public Command adaptiveMovingAimCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate, LightsSubsystemB lightsSubsystem) {
        final double angularTolerance = 0.12;
        final double velocityTolerance = 0.02;

        // This is in theory the inverse of the speed of the note
        final double forwardPredictionCoefficient = 1 / 16.0;

        BooleanSupplier readyToFire = () -> swerveDriveSubsystem.isAtDirectionCommand(angularTolerance, velocityTolerance);

        ProfiledPIDController controller = new ProfiledPIDController(5, 0, .5, new TrapezoidProfile.Constraints(4.5, 8));

        Supplier<Pose2d> predictedPose = () -> {
            Pose2d currentPose = swerveDriveSubsystem.getPose();
            ChassisSpeeds currentSpeed = swerveDriveSubsystem.getFieldRelativeChassisSpeeds();
            double distance = visionSubsystem.getSpeakerDistance(swerveDriveSubsystem.getPose());
            return new Pose2d(
                currentPose.getX() + currentSpeed.vxMetersPerSecond * forwardPredictionCoefficient * distance,
                currentPose.getY() + currentSpeed.vyMetersPerSecond * forwardPredictionCoefficient * distance,
                new Rotation2d(currentPose.getRotation().getRadians()));
        };

        Command aimAtTag = swerveDriveSubsystem.directionCommand(() -> visionSubsystem.getSpeakerAngle(predictedPose.get()).plus(new Rotation2d(Math.PI)), 
            forward, strafe, controller);

        Command spinUpCommand = shooterSubsystem.shootCommand(() -> visionSubsystem.getSpeakerDistance(predictedPose.get()));

        Command lightsCommand = run(() -> {
                if (readyToFire.getAsBoolean()) {
                    LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.green);
                } else {
                    LightsSubsystemB.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystemB.yellow, 0.3);
                }
            }, lightsSubsystem).asProxy();

        return parallel(aimAtTag, spinUpCommand, lightsCommand, run(() -> {readyToFire.getAsBoolean();}));
    }

    public Command movingAimCommandAuto() {
        final double angularTolerance = 0.06;
        final double velocityTolerance = 0.02;

        BooleanSupplier readyToFire = () -> swerveDriveSubsystem.isAtDirectionCommand(angularTolerance, velocityTolerance);

        Command aimAtTag = swerveDriveSubsystem.directionCommandAuto(() -> visionSubsystem.getSpeakerAngle(swerveDriveSubsystem.getPose()).plus(new Rotation2d(Math.PI)));

        Command spinUpCommand = shooterSubsystem.shootCommand(() -> visionSubsystem.getSpeakerDistance(swerveDriveSubsystem.getPose()));

        return parallel(aimAtTag, spinUpCommand.asProxy(), run(() -> {readyToFire.getAsBoolean();}));
    }

    public Command spinupCommand() {
        Command spinUpCommand = shooterSubsystem.shootCommand(() -> visionSubsystem.getSpeakerDistance(swerveDriveSubsystem.getPose()));

        return spinUpCommand.asProxy();
    }

}
