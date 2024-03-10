package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import frc.lib.vision.LimelightRawAngles;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TrapConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AimAndShootCommands;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.subsystems.climber.ClimberIOFalcon;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeIOFalconRedline;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
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
import frc.robot.subsystems.vision.AprilTagIOPhotonVisionPinhole;
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
                var camera = new PhotonCamera("LeftCamera");
                if (VisionConstants.usingPinholeModel) {
                    leftCamera = new AprilTagIOPhotonVisionPinhole(
                    camera, Constants.VisionConstants.robotToLeftCamera, () -> swerveDriveSubsystem.getRotation());
                } else {
                    leftCamera = new AprilTagIOPhotonVision(
                        camera, Constants.VisionConstants.robotToLeftCamera);
                }
            } catch (Exception e) {
                System.err.print(e);
                leftCamera = new AprilTagIOSim();
            }
            try {
                var camera = new PhotonCamera("RightCamera");
                if (VisionConstants.usingPinholeModel) {
                    rightCamera = new AprilTagIOPhotonVisionPinhole(
                    camera, Constants.VisionConstants.robotToLeftCamera, () -> swerveDriveSubsystem.getRotation());
                } else {
                    rightCamera = new AprilTagIOPhotonVision(
                        camera, Constants.VisionConstants.robotToRightCamera);
                }
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
                new Trigger(() -> intakeSubsystem.hasPieceRaw());
        hasPieceTrigger.onTrue(runOnce(() -> Logger.log("/Robot/Has Game Piece", true)));
        hasPieceTrigger.onFalse(runOnce(() -> Logger.log("/Robot/Has Game Piece", false)));

        // the negates are in place so it debounces falling edges only. the .debounce decorator only debonces rising edges by default 
        // (the oppositie of what we want if it keeps randomly flickering off)
        lightsSubsystem.setHasPieceSupplier(() -> intakeSubsystem.hasPieceSmoothed());

        /* Set right joystick bindings ]
         * 
        */
        rightDriveController.getLeftTopRight().onTrue(runOnce(() -> swerveDriveSubsystem.seedFieldRelative(new Pose2d()), swerveDriveSubsystem));
        rightDriveController
                .getLeftTopLeft()
                .onTrue(runOnce(() -> {
                    Pose2d currentPose = swerveDriveSubsystem.getPose();
                    Pose2d nextPose = new Pose2d(currentPose.getTranslation(), new Rotation2d(FieldConstants.isBlue() ? 0 : Math.PI));
                    swerveDriveSubsystem.seedFieldRelative(nextPose);
                }, swerveDriveSubsystem));
        rightDriveController.nameLeftTopLeft("Reset Gyro Angle");

        rightDriveController
                .nameTrigger("Shoot");


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
                .whileTrue(shooterSubsystem.shootCommand(new ShooterState(.65,.65,Rotation2d.fromDegrees(35))));

        rightDriveController.getBottomThumb().and(rightDriveController.getTrigger())
            .whileTrue(intakeSubsystem.shootCommand());

        rightDriveController
                .getLeftThumb().and(rightDriveController.getTrigger().negate())
                .whileTrue(intakeSubsystem.intakeCommand());

        (rightDriveController
                .getRightThumb().and(rightDriveController.getTrigger())).debounce(0.2, DebounceType.kFalling)
                .whileTrue(parallel(intakeSubsystem.shooterIntakeCommand(), shooterSubsystem.shootCommand(ShooterState.fromVoltages(-.25,-.25,Rotation2d.fromDegrees(55)))).until(() -> intakeSubsystem.getRollerSensor())
            );
        
        (rightDriveController
            .getLeftThumb().and(rightDriveController.getTrigger())).whileTrue(mlIntakeStraightCommand())
                .whileTrue(intakeSubsystem.intakeCommand());
        
        
            

        rightDriveController
                .getRightBottomLeft().whileTrue(ampScoreCommand());
        
        operatorController
                .getLeftBumper().whileTrue(shooterSubsystem.shootCommand(new ShooterState(.05,.2,Rotation2d.fromDegrees(55))));
            
        operatorController
                .getLeftBumper().and(rightDriveController.getTrigger()).whileTrue(intakeSubsystem.ampCommand());

        rightDriveController
            .getRightTopRight().whileTrue(
                swerveDriveSubsystem.directionCommand(() -> {
                    Optional<LimelightRawAngles> direction = visionSubsystem.getDetectorInfo();
                    if (direction.isPresent()) {
                        return swerveDriveSubsystem.getRotation().plus(Rotation2d.fromDegrees(-direction.get().ty() / 2.0));
                    } else {
                        return swerveDriveSubsystem.getRotation();
                    }
                }, this::getDriveForwardAxis, this::getDriveStrafeAxis, new ProfiledPIDController(1, 0, .5, new TrapezoidProfile.Constraints(4, 4)))
            );

        rightDriveController
                .getRightTopLeft().whileTrue(
                    mlIntakeCommand()
                );
        
        rightDriveController
                .getRightTopMiddle().whileTrue(
                    mlIntakeStraightCommand()
                );

        /*Set left joystic bindings */
        
        leftDriveController
                .getTrigger()
                .whileTrue(aimAndShootCommands.movingAimCommand(
                    this::getDriveForwardAxis, this::getDriveStrafeAxis, this::getDriveRotationAxis, lightsSubsystem));
        leftDriveController
                .nameTrigger("Aim");

        // leftDriveController
        //         .getTrigger().and(operatorController.getLeftBumper())
        //         .whileTrue(aimAndShootCommands.adaptiveMovingAimCommand(
        //             this::getDriveForwardAxis, this::getDriveStrafeAxis, this::getDriveRotationAxis, lightsSubsystem
        //         ));
        
        leftDriveController
                .getTrigger().and(rightDriveController.getTrigger())
                .whileTrue(intakeSubsystem.shootCommand());
                //.whileTrue(waitUntil(() -> shooterSubsystem.isShooterAtPosition() && swerveDriveSubsystem.isAtDirectionCommand(0.06, 0.02)).andThen(intakeSubsystem.shootCommand()))
;//stoppedShootAndAimCommand());

        leftDriveController
                .getLeftThumb()
                .whileTrue(climberSubsystem.setVoltage(-12));
        
        leftDriveController
                .getRightThumb()
                .whileTrue(climberSubsystem.setVoltage(12));
    
        
        LoggedReceiver topRollerSpeedTunable = Logger.tunable("/ShooterSubsystem/topTunable", .4d);
        LoggedReceiver bottomRollerSpeedTunable = Logger.tunable("/ShooterSubsystem/bottomTunable", .7d);
        LoggedReceiver pivotAngleTunable = Logger.tunable("/ShooterSubsystem/pivotTunable", 56d);
        LoggedReceiver isVoltageBasedTunable = Logger.tunable("/ShooterSubsystem/voltageTunable", false);
        
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

        leftDriveController.getLeftTopRight().onTrue(shooterSubsystem.zeroShooterAngleCommand(Rotation2d.fromDegrees(46)));
        leftDriveController.getLeftBottomRight().onTrue(trapSubsystem.zeroRackPositionCommand());

        leftDriveController.getRightTopMiddle().onTrue(runOnce(() -> visionSubsystem.usingVision = false));
        leftDriveController.getRightTopRight().onTrue(runOnce(() -> visionSubsystem.usingVision = true));
        
        leftDriveController.getRightBottomMiddle().onTrue(runOnce(() -> shooterSubsystem.inPositionDisableMode = true));
        leftDriveController.getRightBottomRight().onTrue(runOnce(() -> shooterSubsystem.inPositionDisableMode = false));

        // Trap Command
        operatorController.getY().whileTrue(trapSubsystem.shootCommand(new TrapState(0,0,34)));

        // Trap Amp Command
        operatorController.getX().whileTrue(trapSubsystem.shootCommand(new TrapState(0,0,24.0)));

        // Trap Source Command
        operatorController.getB().whileTrue(trapSubsystem.shootCommand(new TrapState(6,6,24.0)));

        //Trap Bottom Command (this is not zero to reduce banging. it will slowly glide down if it is below 2.5 instead of stalling)
        operatorController.getA().whileTrue(trapSubsystem.shootCommand(new TrapState(0,0,0)));


        // Trap Eject Comand
        //operatorController.getDPadLeft().whileTrue(trapSubsystem.runIntakeCommand(-12.0, -12.0));

        LoggedReceiver traptopRollerSpeedTunable = Logger.tunable("/TrapSubsystem/topTunable", 3.5d);
        LoggedReceiver trapbottomRollerSpeedTunable = Logger.tunable("/TrapSubsystem/bottomTunable", -3.5d);
        operatorController.getDPadLeft()
                .whileTrue(trapSubsystem.shootCommand(() -> TrapState.fromVoltages(traptopRollerSpeedTunable.getDouble(), trapbottomRollerSpeedTunable.getDouble(), trapSubsystem.holdingVoltage)));

        // Trap Intake Command
        operatorController.getDPadRight().whileTrue(trapSubsystem.runIntakeCommand(9.0, -9.0));

        // Trap Rotate Note Up
        operatorController.getDPadUp().and(operatorController.getRightBumper().negate()).whileTrue(trapSubsystem.runIntakeCommand(-3.5, 3.5));

        // Trap Rotate Note Down
        operatorController.getDPadDown().and(operatorController.getRightBumper().negate()).whileTrue(trapSubsystem.runIntakeCommand(-1.0, 3.0));

        // Shooter Adjustment Up
        operatorController.getDPadUp().and(operatorController.getRightBumper()).onTrue(shooterSubsystem.adjustPitchCorrectionCommand(Rotation2d.fromDegrees(.25)));

        // Shooter Adjustment Down
        operatorController.getDPadDown().and(operatorController.getRightBumper()).onTrue(shooterSubsystem.adjustPitchCorrectionCommand(Rotation2d.fromDegrees(-.25)));

        // Ground Intaking Indicator (Strobe Purple)
        operatorController.getRightTrigger().and(operatorController.getLeftTrigger().negate()).whileTrue(run(() -> LightsSubsystemB.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystemB.purple, 0.3), lightsSubsystem));
        
        // Trap Intaking Indicator (Flash Red and Green)
        operatorController.getLeftTrigger().and(operatorController.getRightTrigger().negate()).whileTrue(repeatingSequence(
            run(() -> LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.red), lightsSubsystem).withTimeout(.1),
            run(() -> LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.green), lightsSubsystem).withTimeout(.1)
        ));

        // Shooter Intaking Indicator (Flash Blue and Yellow)
        operatorController.getLeftTrigger().and(operatorController.getRightTrigger()).whileTrue(
            repeatingSequence(
                run(() -> LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.blue), lightsSubsystem).withTimeout(.1),
                run(() -> LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.yellow), lightsSubsystem).withTimeout(.1)
            )
        );

        //operatorController.getA().onTrue(shooterSubsystem.adjustPitchCorrectionCommand(Rotation2d.fromDegrees(0.5)));
        //operatorController.getB().onTrue(shooterSubsystem.adjustPitchCorrectionCommand(Rotation2d.fromDegrees(0.5)));

        operatorController.getBack().whileTrue(climberSubsystem.moveClimberUpOperator());
        
        operatorController.getStart().toggleOnTrue(run(() -> LightsSubsystemB.LEDSegment.MainStrip.setRainbowAnimation(1), lightsSubsystem));

        // operatorController
        //         .getBack()
        //         .whileTrue(climberSubsystem.setposi(12));

        rightDriveController.sendButtonNamesToNT();
        leftDriveController.sendButtonNamesToNT();
        operatorController.sendButtonNamesToNT();
    }

    public Command ampScoreCommand() {
        DriveToPositionCommand driveToPosition = new DriveToPositionCommand(swerveDriveSubsystem, () -> FieldConstants.getAmpPose().plus(new Transform2d(.7, -.1, new Rotation2d())), false);
        Debouncer debouncer = new Debouncer(0.5);
        return Commands.parallel(driveToPosition);//, shooterSubsystem.ampCommand(), Commands.waitUntil(() -> debouncer.calculate(driveToPosition.atGoal())).andThen(intakeSubsystem.shootCommand()));
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

    public Command mlIntakeCommand() {
        final double aimingFactor = .6; //up to 1 the pose converges. reduces overshoot chance if less than 1;
        final double speed = 1;
        final LinearFilter myFilter = LinearFilter.singlePoleIIR(0.1,0.02);
        Supplier<Rotation2d> angleOfGamepiece = () -> {if(visionSubsystem.getDetectorInfo().isPresent())
                                                            return Rotation2d.fromDegrees(-visionSubsystem.getDetectorInfo().get().ty() * aimingFactor);
                                                        else
                                                            return new Rotation2d();};

        Supplier<Rotation2d> absoluteTargetAngle = () -> Rotation2d.fromRotations(myFilter.calculate(angleOfGamepiece.get().plus(swerveDriveSubsystem.getRotation()).getRotations()));
        ProfiledPIDController omegaController = 
            new ProfiledPIDController(2, 0, 1, new TrapezoidProfile.Constraints(4, 4));
        DoubleSupplier forward = () -> {if(visionSubsystem.getDetectorInfo().isPresent()) 
                                            return absoluteTargetAngle.get().getCos() * speed;
                                        else
                                            return 0;};
        DoubleSupplier strafe = () -> {if(visionSubsystem.getDetectorInfo().isPresent())
                                            return absoluteTargetAngle.get().getSin() * speed;
                                        else
                                            return 0;};

        return swerveDriveSubsystem.directionCommand(absoluteTargetAngle, forward, strafe, omegaController);
    }

    public Command autoMLIntakeTurnCommand() {
        return swerveDriveSubsystem.directionCommandAuto(() -> {
            Optional<LimelightRawAngles> direction = visionSubsystem.getDetectorInfo();
            if (direction.isPresent()) {
                return swerveDriveSubsystem.getRotation().plus(Rotation2d.fromDegrees(-direction.get().ty() * 0.7));
            } else {
                return swerveDriveSubsystem.getRotation();
            }
        }).until(() -> intakeSubsystem.hasPieceSmoothed());
    }

    public Command mlIntakeStraightCommand() {
        SlewRateLimiter forwardSlewer = new SlewRateLimiter(8);
        SlewRateLimiter strafeSlewer = new SlewRateLimiter(8);
        final double aimingFactor = 0.8; //up to 1 the pose converges. reduces overshoot chance if less than 1;
        DoubleSupplier speed = () -> intakeSubsystem.hasPieceSmoothed() ? 0 : 3;
        final LinearFilter myFilter = LinearFilter.singlePoleIIR(0.1,0.02);
        Supplier<Rotation2d> angleOfGamepiece = () -> {if(visionSubsystem.getDetectorInfo().isPresent())
                                                            return Rotation2d.fromDegrees(-visionSubsystem.getDetectorInfo().get().ty() * aimingFactor);
                                                        else
                                                            return new Rotation2d();};

        Supplier<Rotation2d> absoluteTargetAngle = () -> Rotation2d.fromRotations(myFilter.calculate(angleOfGamepiece.get().plus(swerveDriveSubsystem.getRotation()).getRotations()));
        ProfiledPIDController omegaController = 
            new ProfiledPIDController(4, 0, 1, new TrapezoidProfile.Constraints(6, 6));
        DoubleSupplier forward = () -> forwardSlewer.calculate(swerveDriveSubsystem.getRotation().getCos() * speed.getAsDouble());
        DoubleSupplier strafe = () -> strafeSlewer.calculate(swerveDriveSubsystem.getRotation().getSin() * speed.getAsDouble());

        return swerveDriveSubsystem.directionCommand(absoluteTargetAngle, forward, strafe, omegaController).alongWith(runOnce(() -> {
            ChassisSpeeds speeds = swerveDriveSubsystem.getFieldRelativeChassisSpeeds();
            forwardSlewer.calculate(speeds.vxMetersPerSecond);
            strafeSlewer.calculate(speeds.vyMetersPerSecond);
        }));
    }

    public double getDriveForwardAxis() {
        return (FieldConstants.isBlue() ? 1 : -1) *
        -square(deadband(leftDriveController.getYAxis().getRaw(), .15)) * Constants.SwerveConstants.maxSpeed; // );
    }

    public double getDriveStrafeAxis() {
        return (FieldConstants.isBlue() ? 1 : -1) *
        -square(deadband(leftDriveController.getXAxis().getRaw(), .15)) * Constants.SwerveConstants.maxSpeed; // );
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



