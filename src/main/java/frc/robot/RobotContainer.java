package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.lights.LightsIOBlinkin;
import frc.robot.subsystems.lights.LightsIOSim;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.shooter.PivotIOSim;
import frc.robot.subsystems.shooter.RollerIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.AprilTagIOPhotonVision;
import frc.robot.subsystems.vision.AprilTagIOSim;
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

    public AutonomousManager autonomousManager;

    public RobotContainer(TimedRobot robot) {
        if (Robot.isReal()) {
            swerveDriveSubsystem = TunerConstants.DriveTrain;
            lightsSubsystem = new LightsSubsystem(new LightsIOBlinkin(0));
            shooterSubsystem = new ShooterSubsystem(new RollerIOSim(), new RollerIOSim(), new PivotIOSim(), Constants.ShooterConstants.topRollerMap(), Constants.ShooterConstants.bottomRollerMap(), Constants.ShooterConstants.shooterAngleMap());
            visionSubsystem = new VisionSubsystem(
                swerveDriveSubsystem, 
                new AprilTagIOPhotonVision(
                    new PhotonCamera("LeftCamera"), Constants.VisionConstants.robotToLeftCamera),
                new AprilTagIOPhotonVision(
                    new PhotonCamera("RightCamera"), Constants.VisionConstants.robotToRightCamera),
                new PositionTargetIOLimelight());
        } else {
            swerveDriveSubsystem = TunerConstants.DriveTrain;
            lightsSubsystem = new LightsSubsystem(new LightsIOSim());
            shooterSubsystem = new ShooterSubsystem(new RollerIOSim(), new RollerIOSim(), new PivotIOSim(), Constants.ShooterConstants.topRollerMap(), Constants.ShooterConstants.bottomRollerMap(), Constants.ShooterConstants.shooterAngleMap());
            visionSubsystem = new VisionSubsystem(swerveDriveSubsystem, new AprilTagIOSim(), new AprilTagIOSim(), new PositionTargetIOSim() );
        }

        autonomousManager = new AutonomousManager(this);

        configureBindings();
    }

    private void configureBindings() {

        /* Set default commands */
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                this::getDriveForwardAxis, this::getDriveStrafeAxis, this::getDriveRotationAxis));

        /* Set left joystick bindings */
        leftDriveController.getLeftTopLeft().onTrue(runOnce(swerveDriveSubsystem::tareEverything, swerveDriveSubsystem));
        leftDriveController
                .getLeftTopRight()
                .onTrue(runOnce(swerveDriveSubsystem::tareEverything, swerveDriveSubsystem));
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
                .whileTrue(shooterSubsystem.shootCommand(() -> swerveDriveSubsystem.getPose().getTranslation().getNorm()));
        rightDriveController
                .nameTrigger("Shoot");

        rightDriveController
                .getBottomThumb()
                .whileTrue(swerveDriveSubsystem.pathfindToPoseCommand(new Pose2d(2.9, 5.5, new Rotation2d(Math.PI))));

        leftDriveController
                .getBottomThumb()
                .whileTrue(swerveDriveSubsystem.pathfindToPoseCommand(new Pose2d(1.53, 2.557, new Rotation2d(Math.PI))));

        rightDriveController.sendButtonNamesToNT();
        leftDriveController.sendButtonNamesToNT();
        operatorController.sendButtonNamesToNT();
    }

    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand();
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
}
