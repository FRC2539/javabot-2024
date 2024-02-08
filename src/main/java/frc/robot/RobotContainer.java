package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.intake.IntakeIOFalconRedline;
import frc.robot.subsystems.intake.IntakeIOFalconRedlineDupe;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lights.LightsIOBlinkin;
import frc.robot.subsystems.lights.LightsIOSim;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.shooter.PivotIOSim;
import frc.robot.subsystems.shooter.RollerIOSim;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.AprilTagIO;
import frc.robot.subsystems.vision.AprilTagIOPhotonVision;
import frc.robot.subsystems.vision.AprilTagIOSim;
import frc.robot.subsystems.vision.PositionTargetIO;
import frc.robot.subsystems.vision.PositionTargetIOLimelight;
import frc.robot.subsystems.vision.PositionTargetIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

    private final ThrustmasterJoystick leftDriveController =
            new ThrustmasterJoystick(ControllerConstants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController =
            new ThrustmasterJoystick(ControllerConstants.RIGHT_DRIVE_CONTROLLER);
    private final LogitechController operatorController =
            new LogitechController(ControllerConstants.OPERATOR_CONTROLLER);

    public static SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(35, -35, 0);
    public static SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(35, -35, 0);

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private LightsSubsystem lightsSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public AutonomousManager autonomousManager;

    public RobotContainer(TimedRobot robot) {
        if (Robot.isReal()) {
            swerveDriveSubsystem = TunerConstants.DriveTrain;
            lightsSubsystem = new LightsSubsystem(new LightsIOBlinkin(0));
            shooterSubsystem = new ShooterSubsystem(new RollerIOSim(), new RollerIOSim(), new PivotIOSim(), Constants.ShooterConstants.topRollerMap(), Constants.ShooterConstants.bottomRollerMap(), Constants.ShooterConstants.shooterAngleMap());
            
            AprilTagIO leftCamera;
            AprilTagIO rightCamera;
            PositionTargetIO limelight;

            //This silly thing is so that if a camera doesn't connect, the robot still runs.
            try {
                leftCamera = new AprilTagIOPhotonVision(
                    new PhotonCamera("LeftCamera"), Constants.VisionConstants.robotToLeftCamera);
            } catch (Exception e) {
                System.err.print(e);
                leftCamera = new AprilTagIOSim();
            }
            try {
                rightCamera = new AprilTagIOPhotonVision(
                    new PhotonCamera("RightCamera"), Constants.VisionConstants.robotToRightCamera);
            } catch (Exception e) {
                System.err.print(e);
                rightCamera = new AprilTagIOSim();
            }
            try {
                limelight = new PositionTargetIOLimelight();
            } catch (Exception e) {
                System.err.print(e);
                limelight = new PositionTargetIOSim();
            }
            visionSubsystem = new VisionSubsystem(swerveDriveSubsystem, leftCamera, rightCamera, limelight);
            intakeSubsystem = new IntakeSubsystem(new IntakeIOFalconRedline());
        
        } else {
            swerveDriveSubsystem = TunerConstants.DriveTrain;
            lightsSubsystem = new LightsSubsystem(new LightsIOSim());
            shooterSubsystem = new ShooterSubsystem(new RollerIOSim(), new RollerIOSim(), new PivotIOSim(), Constants.ShooterConstants.topRollerMap(), Constants.ShooterConstants.bottomRollerMap(), Constants.ShooterConstants.shooterAngleMap());
            visionSubsystem = new VisionSubsystem(swerveDriveSubsystem, new AprilTagIOSim(), new AprilTagIOSim(), new PositionTargetIOSim() );
            intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
        }

        autonomousManager = new AutonomousManager(this);

        configureBindings();
    }

    private void configureBindings() {

        /* Set default commands */
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                this::getDriveForwardAxis, this::getDriveStrafeAxis, this::getDriveRotationAxis));

        /* Set left joystick bindings */
        leftDriveController.getLeftTopLeft().onTrue(runOnce(() -> swerveDriveSubsystem.seedFieldRelative(new Pose2d()), swerveDriveSubsystem));
        leftDriveController
                .getLeftTopRight()
                .onTrue(runOnce(swerveDriveSubsystem::seedFieldRelative, swerveDriveSubsystem));
        leftDriveController.nameLeftTopLeft("Reset Gyro Angle");


        leftDriveController.getLeftBottomMiddle().whileTrue(runOnce(() -> swerveDriveSubsystem.setControl(new SwerveRequest.SwerveDriveBrake()), swerveDriveSubsystem));
        leftDriveController.nameLeftBottomMiddle("Lock Wheels");

        // Cardinal drive commands (inverted since the arm is on the back of the robot)
        rightDriveController
                .getPOVUp()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(0), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController
                .getPOVRight()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(-90), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController
                .getPOVDown()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(180), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController
                .getPOVLeft()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(90), this::getDriveForwardAxis, this::getDriveStrafeAxis));

        rightDriveController
                .getTrigger()
                .whileTrue(stoppedShootAndAimCommand());
        rightDriveController
                .nameTrigger("Shoot");

        rightDriveController
                .getRightThumb()
                .whileTrue(intakeSubsystem.disabledCommand());

        rightDriveController
                .getBottomThumb()
                .whileTrue(intakeSubsystem.intakeCommand());

        rightDriveController
                .getLeftThumb()
                .whileTrue(intakeSubsystem.ejectCommand());
        
        leftDriveController
                .getTrigger()
                .whileTrue(movingAimCommand());
        leftDriveController
                .nameTrigger("Aim");
        
        LoggedReceiver topRollerSpeedTunable = Logger.tunable("/ShooterSubsystem/topTunable", 0d);
        LoggedReceiver bottomRollerSpeedTunable = Logger.tunable("/ShooterSubsystem/bottomTunable", 0d);
        LoggedReceiver pivotAngleTunable = Logger.tunable("/ShooterSubsystem/pivotTunable", 0d);
        LoggedReceiver isVoltageBasedTunable = Logger.tunable("/ShooterSubsystem/voltageTunable", true);
        leftDriveController
                .getBottomThumb()
                .whileTrue(shooterSubsystem.shootCommand(() -> new ShooterState(
                    topRollerSpeedTunable.getDouble(),
                    bottomRollerSpeedTunable.getDouble(),
                    pivotAngleTunable.getDouble(),
                    isVoltageBasedTunable.getBoolean())));

        rightDriveController.sendButtonNamesToNT();
        leftDriveController.sendButtonNamesToNT();
        operatorController.sendButtonNamesToNT();
    }

    public Command stoppedShootAndAimCommand() {
        return stoppedShootAndAimCommand(Optional.empty());
    }

    public Command stoppedShootAndAimCommand(Optional<Double> timeout) {
        final double angularTolerance = 0.015;
        final double velocityTolerance = 0.01;

        BooleanSupplier readyToFire = () -> (swerveDriveSubsystem.isAtDirectionCommand(angularTolerance, velocityTolerance));

        ProfiledPIDController controller = new ProfiledPIDController(10, 0, .5, new TrapezoidProfile.Constraints(4.5, 8));

        Command aimAtTag = swerveDriveSubsystem.directionCommand(() -> visionSubsystem.getSpeakerAngle(swerveDriveSubsystem.getPose()), 
            () -> 0, () -> 0, controller, true);

        

        Command spinUpCommand = shooterSubsystem.shootCommand(() -> visionSubsystem.getSpeakerDistance(swerveDriveSubsystem.getPose()));

        Command runBeltCommand;

        if (timeout.isEmpty()) {
            runBeltCommand = intakeSubsystem.shootCommand();
        } else {
            runBeltCommand = intakeSubsystem.shootCommand().withTimeout(timeout.get());
        }

        return deadline(waitUntil(readyToFire).andThen(runBeltCommand.asProxy()), aimAtTag, spinUpCommand.asProxy());
    }

    public Command movingAimCommand() {
        final double angularTolerance = 0.015;
        final double velocityTolerance = 0.01;

        BooleanSupplier readyToFire = () -> swerveDriveSubsystem.isAtDirectionCommand(angularTolerance, velocityTolerance);

        ProfiledPIDController controller = new ProfiledPIDController(5, 0, .5, new TrapezoidProfile.Constraints(4.5, 8));

        Command aimAtTag = swerveDriveSubsystem.directionCommand(() -> visionSubsystem.getSpeakerAngle(swerveDriveSubsystem.getPose()), 
            this::getDriveForwardAxis, this::getDriveStrafeAxis, controller);

        Command spinUpCommand = shooterSubsystem.shootCommand(() -> visionSubsystem.getSpeakerDistance(swerveDriveSubsystem.getPose()));

        return parallel(aimAtTag, spinUpCommand, run(() -> {readyToFire.getAsBoolean();}));
    }

    public Command movingAimCommandAuto() {
        final double angularTolerance = 0.015;
        final double velocityTolerance = 0.01;

        BooleanSupplier readyToFire = () -> swerveDriveSubsystem.isAtDirectionCommand(angularTolerance, velocityTolerance);

        Command aimAtTag = swerveDriveSubsystem.directionCommandAuto(() -> visionSubsystem.getSpeakerAngle(swerveDriveSubsystem.getPose()));

        Command spinUpCommand = shooterSubsystem.shootCommand(() -> visionSubsystem.getSpeakerDistance(swerveDriveSubsystem.getPose()));

        return parallel(aimAtTag, spinUpCommand.asProxy(), run(() -> {readyToFire.getAsBoolean();}));
    }


    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand();
    }

    public Command autoIntakeCommand() {
        final double speed = 1;
        Supplier<Rotation2d> angleOfGamepiece = () -> new Rotation2d(); 
        ProfiledPIDController omegaController = 
            new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        DoubleSupplier forward = () -> angleOfGamepiece.get().getCos() * speed;
        DoubleSupplier strafe = () -> angleOfGamepiece.get().getSin() * speed;

        return swerveDriveSubsystem.directionCommand(angleOfGamepiece, forward, strafe, omegaController);
    }

    public double getDriveForwardAxis() {
        return // forwardRateLimiter.calculate(
        -square(deadband(leftDriveController.getYAxis().getRaw(), 0.05)) * Constants.SwerveConstants.maxSpeed; // );
    }

    public double getDriveStrafeAxis() {
        return // strafeRateLimiter.calculate(
        -square(deadband(leftDriveController.getXAxis().getRaw(), 0.05)) * Constants.SwerveConstants.maxSpeed; // );
    }

    public double getDriveRotationAxis() {
        return -cube(deadband(rightDriveController.getXAxis().getRaw(), 0.05))
                * Constants.SwerveConstants.maxAngularVelocity
                * 0.5;
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance) return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    private static double cube(double value) {
        return Math.copySign(value * value * value, value);
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDriveSubsystem;
    }

    public LightsSubsystem getLightsSubsystem() {
        return lightsSubsystem;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }
}



