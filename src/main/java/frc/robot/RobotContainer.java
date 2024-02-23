package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TrapConstants;
import frc.robot.commands.AimAndShootCommands;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOFalcon;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeIOFalconRedline;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lights.LightsIOBlinkin;
import frc.robot.subsystems.lights.LightsIOSim;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.lights.LightsSubsystemB;
import frc.robot.subsystems.shooter.PivotIOFalcon;
import frc.robot.subsystems.shooter.PivotIOSim;
import frc.robot.subsystems.shooter.RollerIOFalcon;
import frc.robot.subsystems.shooter.RollerIOSim;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.trap.RackIONeo550;
import frc.robot.subsystems.trap.RackIOSim;
import frc.robot.subsystems.trap.TrapRollerIONeo550;
import frc.robot.subsystems.trap.TrapRollerIOSim;
import frc.robot.subsystems.trap.TrapState;
import frc.robot.subsystems.trap.TrapSubsystem;
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
    private LightsSubsystemB lightsSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TrapSubsystem trapSubsystem;
    private ClimberSubsystem climberSubsystem;

    private AimAndShootCommands aimAndShootCommands;

    public AutonomousManager autonomousManager;

    public RobotContainer(TimedRobot robot) {
        if (Robot.isReal()) {
            swerveDriveSubsystem = TunerConstants.DriveTrain;
            lightsSubsystem = new LightsSubsystemB(); //new LightsSubsystem(new LightsIOBlinkin(0));
            shooterSubsystem = new ShooterSubsystem(new RollerIOFalcon(ShooterConstants.topShooterPort), new RollerIOFalcon(ShooterConstants.bottomShooterPort), new PivotIOFalcon(), Constants.ShooterConstants.topRollerMap(), Constants.ShooterConstants.bottomRollerMap(), Constants.ShooterConstants.shooterAngleMap());
            trapSubsystem = new TrapSubsystem(new TrapRollerIONeo550(TrapConstants.topRollerPort), new TrapRollerIONeo550(TrapConstants.bottomRollerPort), new RackIONeo550());
            climberSubsystem = new ClimberSubsystem(new ClimberIOFalcon());

            AprilTagIO leftCamera;
            AprilTagIO rightCamera;
            PositionTargetIO limelight;
            PositionTargetIO limelightApriltag;

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
                limelight = new PositionTargetIOLimelight("limelight-intake");
            }  catch (Exception e) {
                System.err.print(e);
                limelight = new PositionTargetIOSim();
            }
            
            try {
                limelightApriltag = new PositionTargetIOLimelight("limelight-shooter");

            } catch (Exception e) {
                System.err.print(e);
                limelightApriltag = new PositionTargetIOSim();
            }
            visionSubsystem = new VisionSubsystem(swerveDriveSubsystem, leftCamera, rightCamera, limelight, limelightApriltag);
            intakeSubsystem = new IntakeSubsystem(new IntakeIOFalconRedline());
        
        } else {
            swerveDriveSubsystem = TunerConstants.DriveTrain;
            lightsSubsystem = new LightsSubsystemB();
            shooterSubsystem = new ShooterSubsystem(new RollerIOSim(), new RollerIOSim(), new PivotIOSim(), Constants.ShooterConstants.topRollerMap(), Constants.ShooterConstants.bottomRollerMap(), Constants.ShooterConstants.shooterAngleMap());
            visionSubsystem = new VisionSubsystem(swerveDriveSubsystem, new AprilTagIOSim(), new AprilTagIOSim(), new PositionTargetIOSim(),new PositionTargetIOSim() );
            intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
            trapSubsystem = new TrapSubsystem(new TrapRollerIOSim(), new TrapRollerIOSim(), new RackIOSim());
            climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
        }

        aimAndShootCommands = new AimAndShootCommands(
            swerveDriveSubsystem, visionSubsystem, shooterSubsystem, intakeSubsystem
        );

        autonomousManager = new AutonomousManager(this);

        configureBindings();
    }

    private void configureBindings() {

        /* Set default commands */
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                this::getDriveForwardAxis, this::getDriveStrafeAxis, this::getDriveRotationAxis));

        /*Set non-button, multi-subsystem triggers */

        Trigger hasPieceTrigger =
                new Trigger(() -> intakeSubsystem.hasPiece());
        hasPieceTrigger.onTrue(runOnce(() -> Logger.log("/Robot/Has Game Piece", true)));
        hasPieceTrigger.onFalse(runOnce(() -> Logger.log("/Robot/Has Game Piece", false)));

        new Trigger(() -> intakeSubsystem.hasPiece())
                .debounce(0.05)
                .whileTrue(run(() -> LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.white), lightsSubsystem));

        /* Set right joystick bindings */
        rightDriveController.getLeftTopRight().onTrue(runOnce(() -> swerveDriveSubsystem.seedFieldRelative(new Pose2d()), swerveDriveSubsystem));
        rightDriveController
                .getLeftTopLeft()
                .onTrue(runOnce(swerveDriveSubsystem::seedFieldRelative, swerveDriveSubsystem));
        rightDriveController.nameLeftTopLeft("Reset Gyro Angle");

        rightDriveController
                .nameTrigger("Shoot");

        rightDriveController.getLeftBottomMiddle().whileTrue(runOnce(() -> swerveDriveSubsystem.setControl(new SwerveRequest.SwerveDriveBrake()), swerveDriveSubsystem));
        rightDriveController.nameLeftBottomMiddle("Lock Wheels");

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
                .getRightThumb()
                .whileTrue(intakeSubsystem.ejectCommand());

        rightDriveController
                .getBottomThumb()
                .whileTrue(shooterSubsystem.shootCommand(ShooterState.fromVoltages(.85,.75,Rotation2d.fromDegrees(35))));

        rightDriveController.getBottomThumb().and(rightDriveController.getTrigger())
            .whileTrue(intakeSubsystem.shootCommand());

        rightDriveController
                .getLeftThumb()
                .whileTrue(intakeSubsystem.intakeCommand());
        
        /*Set left joystic bindings */
        
        leftDriveController
                .getTrigger()
                .whileTrue(aimAndShootCommands.movingAimCommand(
                    this::getDriveForwardAxis, this::getDriveStrafeAxis, this::getDriveRotationAxis, lightsSubsystem));
        leftDriveController
                .nameTrigger("Aim");
        
        leftDriveController
                .getTrigger().and(rightDriveController.getTrigger())
                .whileTrue(intakeSubsystem.shootCommand());//stoppedShootAndAimCommand());

        leftDriveController
                .getLeftThumb()
                .whileTrue(climberSubsystem.setVoltage(-12));
        
        leftDriveController
                .getRightThumb()
                .whileTrue(climberSubsystem.setVoltage(12));
    
        
        LoggedReceiver topRollerSpeedTunable = Logger.tunable("/ShooterSubsystem/topTunable", .4d);
        LoggedReceiver bottomRollerSpeedTunable = Logger.tunable("/ShooterSubsystem/bottomTunable", .7d);
        LoggedReceiver pivotAngleTunable = Logger.tunable("/ShooterSubsystem/pivotTunable", 55d);
        LoggedReceiver isVoltageBasedTunable = Logger.tunable("/ShooterSubsystem/voltageTunable", true);
        
        leftDriveController
                .getBottomThumb()
                .whileTrue(shooterSubsystem.shootCommand(() -> new ShooterState(
                    topRollerSpeedTunable.getDouble(),
                    bottomRollerSpeedTunable.getDouble(),
                    Rotation2d.fromDegrees(pivotAngleTunable.getDouble()),
                    isVoltageBasedTunable.getBoolean(),
                    false)));
        
        leftDriveController.getBottomThumb().and(rightDriveController.getTrigger())
            .whileTrue(intakeSubsystem.shootCommand());
        
        leftDriveController.getLeftTopLeft().whileTrue(
            shooterSubsystem.shootCommand(ShooterState.fromVoltages(0, 0, .6)));
        leftDriveController.getLeftBottomLeft().whileTrue(
            shooterSubsystem.shootCommand(ShooterState.fromVoltages(0, 0, -.6)));

        leftDriveController.getRightTopLeft().whileTrue(climberSubsystem.overrideVoltageCommand());
        leftDriveController.getRightBottomLeft().onTrue(climberSubsystem.zeroClimberCommand());
        
        leftDriveController.getLeftBottomMiddle().whileTrue(trapSubsystem.shootCommand(TrapState.fromVoltages(0, 0, 0.0)));
        leftDriveController.getLeftTopMiddle().whileTrue(trapSubsystem.shootCommand(TrapState.fromVoltages(0, 0, 2.4)));

        operatorController.getA().onTrue(shooterSubsystem.zeroShooterAngleCommand(Rotation2d.fromDegrees(46)));
        operatorController.getB().onTrue(shooterSubsystem.updateShooterAngleCommand());



        // 1.2 is about kG
        operatorController.getLeftBumper().whileTrue(trapSubsystem.shootCommand(TrapState.fromVoltages(0, 0, 0)));
        operatorController.getRightBumper().whileTrue(trapSubsystem.shootCommand(TrapState.fromVoltages(0, 0, .9)));
        operatorController.getY().whileTrue(trapSubsystem.shootCommand(new TrapState(0,0,33)));
        operatorController.getX().whileTrue(trapSubsystem.shootCommand(new TrapState(0,0,2)));

        operatorController.getDPadLeft().whileTrue(trapSubsystem.runIntakeCommand(-12.0, -12.0));
        operatorController.getDPadRight().whileTrue(trapSubsystem.runIntakeCommand(6.0, 6.0));
        operatorController.getDPadUp().whileTrue(trapSubsystem.runIntakeCommand(2.0, -2.0));
        operatorController.getDPadDown().whileTrue(trapSubsystem.runIntakeCommand(-2.0, 2.0));

        operatorController.getRightTrigger().whileTrue(run(() -> LightsSubsystemB.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystemB.purple, 0.3), lightsSubsystem));
        operatorController.getLeftTrigger().whileTrue(repeatingSequence(
            run(() -> LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.red), lightsSubsystem).withTimeout(.1),
            run(() -> LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.green), lightsSubsystem).withTimeout(.1)
        ));
        operatorController.getStart().toggleOnTrue(run(() -> LightsSubsystemB.LEDSegment.MainStrip.setRainbowAnimation(1), lightsSubsystem));


        rightDriveController.sendButtonNamesToNT();
        leftDriveController.sendButtonNamesToNT();
        operatorController.sendButtonNamesToNT();
    }

    public Command ampScoreCommand() {
        DriveToPositionCommand driveToPosition = new DriveToPositionCommand(swerveDriveSubsystem, FieldConstants.getAmpPose().plus(new Transform2d(1, 0, new Rotation2d())), false);
        Debouncer debouncer = new Debouncer(0.5);
        return Commands.parallel(driveToPosition, shooterSubsystem.ampCommand(), Commands.waitUntil(() -> debouncer.calculate(driveToPosition.atGoal())).andThen(intakeSubsystem.shootCommand()));
    }

    public Command sourceIntakeCommand() {
        DriveToPositionCommand driveToPosition = new DriveToPositionCommand(swerveDriveSubsystem, FieldConstants.getSourcePose().plus(new Transform2d(1, 0, new Rotation2d())), false);
        Debouncer debouncer = new Debouncer(0.5);
        return Commands.parallel(driveToPosition);
    }

    public Command trapScoringCommand() {
        ArrayList<Pose2d> trapPoses = new ArrayList<Pose2d>();
        trapPoses.add(FieldConstants.getPoseFromTag(FieldConstants.getTrap1Tag()));
        trapPoses.add(FieldConstants.getPoseFromTag(FieldConstants.getTrap2Tag()));
        trapPoses.add(FieldConstants.getPoseFromTag(FieldConstants.getTrap3Tag()));

        Supplier<Pose2d> getNearestPose = () -> {
            Pose2d currentPose = swerveDriveSubsystem.getPose();

            return currentPose.nearest(trapPoses).plus(new Transform2d(1, 0, new Rotation2d()));
        };

        DriveToPositionCommand driveToPosition = new DriveToPositionCommand(swerveDriveSubsystem, getNearestPose, false);
        Debouncer debouncer = new Debouncer(0.5);
        return Commands.parallel(driveToPosition);
    }

    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand();
    }

    public Command autoIntakeCommand() {
        final double speed = 1;
        final LinearFilter myFilter = LinearFilter.singlePoleIIR(0.1,0.02);
        Supplier<Rotation2d> angleOfGamepiece = () -> {if(visionSubsystem.getDetectorInfo().isPresent())
                                                            return Rotation2d.fromDegrees(visionSubsystem.getDetectorInfo().get().tx());
                                                        else
                                                            return new Rotation2d();};

        Supplier<Rotation2d> absoluteTargetAngle = () -> Rotation2d.fromRotations(myFilter.calculate(angleOfGamepiece.get().plus(swerveDriveSubsystem.getRotation()).getRotations()));
        ProfiledPIDController omegaController = 
            new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        DoubleSupplier forward = () -> {if(visionSubsystem.getDetectorInfo().isPresent()) 
                                            return absoluteTargetAngle.get().getCos() * speed;
                                        else
                                            return 0;};
        DoubleSupplier strafe = () -> {if(visionSubsystem.getDetectorInfo().isPresent())
                                            return absoluteTargetAngle.get().getSin() * speed;
                                        else
                                            return 0;};

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

    public LightsSubsystemB getLightsSubsystem() {
        return lightsSubsystem;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public TrapSubsystem getTrapSubsystem() {
        return trapSubsystem;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    public AimAndShootCommands getAimAndShootCommands() {
        return aimAndShootCommands;
    }
}



