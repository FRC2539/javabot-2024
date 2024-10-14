package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils.AnyContainer;
import frc.lib.vision.LimelightRawAngles;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TrapConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AimAndSpinupCommand;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.commands.IntakeAssistCommandComplex;
import frc.robot.subsystems.climber.ClimberIOFalcon;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeIOFalcon;
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
import frc.robot.subsystems.vision.AprilTagIOLimelight3G;
import frc.robot.subsystems.vision.AprilTagIOPhotonVision;
import frc.robot.subsystems.vision.AprilTagIOSim;
import frc.robot.subsystems.vision.PositionTargetIO;
import frc.robot.subsystems.vision.PositionTargetIOLimelight;
import frc.robot.subsystems.vision.PositionTargetIOPhotonVision;
import frc.robot.subsystems.vision.PositionTargetIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import frc.robot.subsystems.amptransport.AmpTransportSubsystem;



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
    private AmpTransportSubsystem  ampTransportSubsystem;
    private TrapSubsystem trapSubsystem;
//     private ShamperSubsystem shamperSubsystem;
    private ClimberSubsystem climberSubsystem;

    public AutonomousManager autonomousManager;

    public VisionSystemSim visionSim;

    public RobotContainer(TimedRobot robot) {
        // This is where all the robot subsystems are initialized.
        // If the robot is real it creates these:
        Mechanism2d mech = new Mechanism2d(0, 0);
        MechanismRoot2d shooterMechRoot =
                mech.getRoot("shooterRoot", Units.inchesToMeters(-2), Units.inchesToMeters(5));
        MechanismRoot2d climberMechRoot = mech.getRoot("climberRoot", Units.inchesToMeters(0), Units.inchesToMeters(5));
        MechanismRoot2d trapMechRoot = mech.getRoot("trapRoot", Units.inchesToMeters(8), Units.inchesToMeters(5));

        MechanismLigament2d shooterMech =
                shooterMechRoot.append(new MechanismLigament2d("shooter", 0.4, 180, 15, new Color8Bit(Color.kBlack)));
        MechanismLigament2d climberMech =
                climberMechRoot.append(new MechanismLigament2d("climber", 0.0, 90, 4, new Color8Bit(Color.kOrange)));
        MechanismLigament2d trapMech =
                trapMechRoot.append(new MechanismLigament2d("trap", 0.4, 70, 4, new Color8Bit(Color.kBlue)));
        
        //fake
        // MechanismLigament2d shamperMech =
        //         trapMechRoot.append(new MechanismLigament2d("shamper", 0.4, 70, 4, new Color8Bit(Color.kBeige)));
        

        if (Robot.isReal()) {
            swerveDriveSubsystem = TunerConstants.DriveTrain;
            lightsSubsystem = new LightsSubsystemB(); // new LightsSubsystem(new LightsIOBlinkin(0));
            shooterSubsystem = new ShooterSubsystem(
                    new RollerIOFalcon(ShooterConstants.topShooterPort),
                    new RollerIOFalcon(ShooterConstants.bottomShooterPort),
                    new PivotIOFalcon(),
                    Constants.ShooterConstants.topRollerMap(),
                    Constants.ShooterConstants.bottomRollerMap(),
                    Constants.ShooterConstants.shooterAngleMap(),
                    shooterMech);
            trapSubsystem = new TrapSubsystem(
                    new TrapRollerIONeo550(TrapConstants.topRollerPort),
                    new TrapRollerIONeo550(TrapConstants.bottomRollerPort),
                    new RackIONeo550(),
                    trapMech);
        //     shamperSubsystem = new ShamperSubsystem(new ShamperIOSim(), shamperMech);
            climberSubsystem = new ClimberSubsystem(new ClimberIOFalcon(), climberMech);

            AprilTagIO limelightAprilTag;

            PositionTargetIO limelightIntake;

            // This silly thing is so that if a camera doesn't connect, the robot still runs. It
            // basically tries to connect and if it cant, creates a simulation camera.
            try {
                if (VisionConstants.usingPinholeModel) {
                    limelightAprilTag = new AprilTagIOLimelight3G(
                            "limelight-april",
                            Constants.VisionConstants.robotToApriltagCamera,
                            () -> swerveDriveSubsystem.getRotation());
                } else {
                    limelightAprilTag = new AprilTagIOLimelight3G(
                            "limelight-april",
                            Constants.VisionConstants.robotToApriltagCamera,
                            () -> swerveDriveSubsystem.getRotation());
                }
            } catch (Exception e) {
                System.err.print(e);
                limelightAprilTag = new AprilTagIOSim();
            }

            try {
                limelightIntake = new PositionTargetIOLimelight("limelight-intake");
            } catch (Exception e) {
                System.err.print(e);
                limelightIntake = new PositionTargetIOSim();
            }

            visionSubsystem = new VisionSubsystem(swerveDriveSubsystem, limelightAprilTag, limelightIntake);
            intakeSubsystem = new IntakeSubsystem(new IntakeIOFalcon());
            ampTransportSubsystem = new AmpTransportSubsystem();

        } else {
            visionSim = new VisionSystemSim("main");
            // If the robot is in simulation it uses these.
            swerveDriveSubsystem = TunerConstants.DriveTrain;
            lightsSubsystem = new LightsSubsystemB();
            shooterSubsystem = new ShooterSubsystem(
                    new RollerIOSim(),
                    new RollerIOSim(),
                    new PivotIOSim(),
                    Constants.ShooterConstants.topRollerMap(),
                    Constants.ShooterConstants.bottomRollerMap(),
                    Constants.ShooterConstants.shooterAngleMap(),
                    shooterMech);

            // Setup vision subsystem simulations
            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(97.65));
            cameraProp.setCalibError(0.4, 0.2);
            cameraProp.setFPS(5);
            cameraProp.setAvgLatencyMs(20);

            // Setup vision subsystem simulations
            SimCameraProperties cameraProp_intake = new SimCameraProperties();
            cameraProp_intake.setCalibration(1280, 800, Rotation2d.fromDegrees(80.46));
            cameraProp_intake.setCalibError(0.4, 0.2);
            cameraProp_intake.setFPS(15);
            cameraProp_intake.setAvgLatencyMs(20);

            PhotonCamera camera_april = new PhotonCamera("limelight-april");
            PhotonCameraSim camera_aprilSim = new PhotonCameraSim(camera_april, cameraProp);

            PhotonCamera camera_intake = new PhotonCamera("limelight-intake");
            PhotonCameraSim camera_intakeSim = new PhotonCameraSim(camera_intake, cameraProp_intake);

            visionSim.addAprilTags(Constants.FieldConstants.aprilTagFieldLayout);

            TargetModel note = new TargetModel(0.3556, 0.3556, 0.0508);

            for (double x = FieldConstants.fieldWidth / 10;
                    x < FieldConstants.fieldWidth;
                    x += FieldConstants.fieldWidth / 5) {
                visionSim.addVisionTargets(new VisionTargetSim(
                        new Pose3d(FieldConstants.fieldLength / 2, x, 0.0254, new Rotation3d()), note));
            }

            visionSim.addCamera(camera_aprilSim, Constants.VisionConstants.robotToApriltagCamera);
            visionSim.addCamera(camera_intakeSim, Constants.VisionConstants.limelightRobotToCamera);

            camera_aprilSim.enableRawStream(true);
            camera_aprilSim.enableProcessedStream(true);
            camera_aprilSim.enableDrawWireframe(true);

            camera_intakeSim.enableRawStream(true);
            camera_intakeSim.enableProcessedStream(true);
            camera_intakeSim.enableDrawWireframe(true);

            visionSubsystem = new VisionSubsystem(
                    swerveDriveSubsystem,
                    new AprilTagIOPhotonVision(camera_april, Constants.VisionConstants.robotToApriltagCamera, true),
                    new PositionTargetIOPhotonVision(camera_intake, true));

            intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
            trapSubsystem = new TrapSubsystem(new TrapRollerIOSim(), new TrapRollerIOSim(), new RackIOSim(), trapMech);
            climberSubsystem = new ClimberSubsystem(new ClimberIOSim(), climberMech);
        }

        SmartDashboard.putData("Mech2d", mech);

        autonomousManager = new AutonomousManager(this);

        DriverStation.getAlliance();
        FieldConstants.isBlue();

        // Creates all bindings to controllers and other triggers
        configureBindings();
    }

    private void configureBindings() {

        /* Set default commands */
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                this::getDriveForwardAxis, this::getDriveStrafeAxis, this::getDriveRotationAxis));

        /*Set non-button, multi-subsystem triggers */
        // the negates are in place so it debounces falling edges only. the .debounce decorator only debonces rising
        // edges by default
        // (the oppositie of what we want if it keeps randomly flickering off)
        lightsSubsystem.setHasPieceSupplier(() -> intakeSubsystem.hasPieceSmoothed() || ampTransportSubsystem.hasPiece());

        /* Set right joystick bindings */

        // Reset Field Position
        rightDriveController
                .getLeftTopRight()
                .onTrue(runOnce(() -> swerveDriveSubsystem.seedFieldRelative(new Pose2d()), swerveDriveSubsystem));

        // Reset Gyro Rotation
        rightDriveController
                .getLeftTopLeft()
                .onTrue(runOnce(
                        () -> {
                            Pose2d currentPose = swerveDriveSubsystem.getPose();
                            Pose2d nextPose = new Pose2d(
                                    currentPose.getTranslation(),
                                    new Rotation2d(FieldConstants.isBlue() ? 0 : Math.PI));
                            swerveDriveSubsystem.seedFieldRelative(nextPose);
                        },
                        swerveDriveSubsystem));

        // Cardinal Angle Drive Commands
        rightDriveController
                .getPOVUp()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(180), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController
                .getPOVRight()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(90), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController
                .getPOVDown()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(0), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController
                .getPOVLeft()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(-90), this::getDriveForwardAxis, this::getDriveStrafeAxis));

        // Eject note from intake
        //rightDriveController.getRightThumb().whileTrue(intakeSubsystem.ejectCommand());
        

        //rightDriveController.getRightThumb().whileTrue(parallel(
          //      intakeSubsystem.curlCommand(), ampTransportSubsystem.ampTransportCommand(.20),trapSubsystem.runIntakeCommand(-9,9)
        //));

        operatorController.getDPadRight().and(operatorController.getRightBumper()).whileTrue(ampTransportSubsystem.ampTransportCommand(-.30));

        //(intakeSubsystem.curlCommand());

        // Podium Shot Spinup
        rightDriveController
                .getBottomThumb()
                // .and(leftDriveController.getBottomThumb().negate())
                .onTrue(shooterSubsystem
                        .shootCommand(() -> ShooterSubsystem.podiumShot)
                        .until(rightDriveController
                                .getBottomThumb()
                                .or(leftDriveController.getBottomThumb())
                                .negate()));

        // rightDriveController
        //         .getBottomThumb()
        //         .and(leftDriveController.getBottomThumb())
        //         .whileTrue(shooterSubsystem.shootCommand(() -> new ShooterState(1, -1, Rotation2d.fromDegrees(15))));

        // Shoot for Podium Shot
        rightDriveController
                .getBottomThumb()
                .and(rightDriveController.getTrigger())
                .whileTrue(intakeSubsystem.shootCommand());

        // Run Intake
        rightDriveController
                .getLeftThumb()
                .and(rightDriveController.getTrigger().negate())
                .whileTrue(intakeSubsystem.intakeCommand());

        
        // Run Shooter Intake
        rightDriveController
                .getRightThumb()
                .and(rightDriveController.getTrigger())
                .whileTrue(parallel(
                        intakeSubsystem.shooterIntakeCommand(),
                        shooterSubsystem.shootCommand(
                                ShooterState.fromVoltages(-.25, -.25, Rotation2d.fromDegrees(55)))));


        // Intake Assist Command
        (rightDriveController.getLeftThumb().and(rightDriveController.getTrigger()))
                .whileTrue(new IntakeAssistCommandComplex(
                                swerveDriveSubsystem,
                                visionSubsystem,
                                lightsSubsystem,
                                this::getDriveForwardAxis,
                                this::getDriveStrafeAxis,
                                this::getDriveRotationAxis)
                        .until(() -> intakeSubsystem.hasPieceRaw()))
                .whileTrue(intakeSubsystem.intakeCommand());

        // Score In Amp (Shooter)

        rightDriveController.getRightBottomLeft().whileTrue(ampScoreCommand());

        // Turn To Note (Limelight)
        rightDriveController
                .getRightTopRight()
                .whileTrue(swerveDriveSubsystem.directionCommand(
                        () -> {
                            Optional<LimelightRawAngles> direction = visionSubsystem.getDetectorInfo();
                            if (direction.isPresent()) {
                                return swerveDriveSubsystem
                                        .getRotation()
                                        .plus(Rotation2d.fromDegrees(
                                                -direction.get().ty() / 2.0));
                            } else {
                                return swerveDriveSubsystem.getRotation();
                            }
                        },
                        this::getDriveForwardAxis,
                        this::getDriveStrafeAxis,
                        new ProfiledPIDController(1, 0, .5, new TrapezoidProfile.Constraints(4, 4))));

        // Turn and Drive To Note
        rightDriveController
                .getRightTopLeft()
                .whileTrue(
                        // mlIntakeCommand()
                        // operatorController.getStart().toggleOnTrue(run(() ->
                        // LightsSubsystemB.LEDSegment.MainStrip.setRainbowAnimation(1), lightsSubsystem));

                        // run(() -> LightsSubsystemB.LEDSegment.MainStrip.setRainbowAnimation(1), lightsSubsystem)
                        mlAimCommand());

        // Turn and Drive to Note but only drive straight
        rightDriveController.getRightTopMiddle().whileTrue(mlIntakeStraightCommand());

        // Turn using gyro for the Podium Shot
        rightDriveController
                .getBottomThumb()
                .whileTrue(Commands.either(
                                swerveDriveSubsystem.cardinalCommand(
                                        () -> new Rotation2d( - 0.31), this::getDriveForwardAxis, this::getDriveStrafeAxis),
                                swerveDriveSubsystem.cardinalCommand(
                                        () -> new Rotation2d(Math.PI + 0.31),
                                        this::getDriveForwardAxis,
                                        this::getDriveStrafeAxis),
                                // shooterSubsystem.shootCommand(new ShooterState(.05,.2,Rotation2d.fromDegrees(55)),
                                FieldConstants::isBlue)
                        .alongWith(Commands.either(
                                run(
                                                () -> {
                                                    LightsSubsystemB.LEDSegment.MainStrip.setColor(
                                                            LightsSubsystemB.green);
                                                },
                                                lightsSubsystem)
                                        .asProxy(),
                                run(() -> {}),
                                () -> swerveDriveSubsystem.isAtDirectionCommand(0.1, 0.2))));

        /*Set left joystic bindings */

        // Aim and Spinup Using Vision
        Command stoppedShootAimAndSpinup = new AimAndSpinupCommand(
                swerveDriveSubsystem,
                shooterSubsystem,
                lightsSubsystem,
                visionSubsystem,
                () -> getDriveForwardAxis(),
                () -> getDriveStrafeAxis(),
                () -> 0,
                false,
                0,
                0,
                true,
                true,
                true,
                false,
                false,false);

        AimAndSpinupCommand movingShootAimAndSpinup = new AimAndSpinupCommand(
                swerveDriveSubsystem,
                shooterSubsystem,
                lightsSubsystem,
                visionSubsystem,
                () -> getDriveForwardAxis(),
                () -> getDriveStrafeAxis(),
                () -> 0,
                false,
                .75 / 16.0,
                0.25,
                true,
                true,
                false,
                false,
                false,false);

                Command autoShootingCommand;
                {
                    AimAndSpinupCommand aimAndSpinupCommand = new AimAndSpinupCommand(
                            swerveDriveSubsystem,
                            shooterSubsystem,
                            lightsSubsystem,
                            visionSubsystem,
                            () -> 0,
                            () -> 0,
                            () -> 0,
                            false,
                            0,
                            0,
                            true,
                            true,
                            true,
                            true,
                            false,
                            false);
                    autoShootingCommand = Commands.deadline(
                            Commands.waitSeconds(0.5)
                                    .andThen(waitUntil(() -> aimAndSpinupCommand.isAtAngleAndSpunUpAndTarget())
                                            .withTimeout(2.0))
                                    .andThen(intakeSubsystem.shootCommand().asProxy().withTimeout(0.5)),
                            aimAndSpinupCommand,
                            run(() -> {}, swerveDriveSubsystem),
                            run(() -> {}, shooterSubsystem).asProxy());
                }

        Command autoShootingCommandSpinup;
                {
                    AimAndSpinupCommand aimAndSpinupCommand = new AimAndSpinupCommand(
                            swerveDriveSubsystem,
                            shooterSubsystem,
                            lightsSubsystem,
                            visionSubsystem,
                            this::getDriveForwardAxis,
                            this::getDriveStrafeAxis,
                            this::getDriveRotationAxis,
                            false,
                            0,
                            0,
                            true,
                            false,
                            true,
                            true,
                            false,
                            false);
                    autoShootingCommandSpinup = Commands.deadline(
                            aimAndSpinupCommand,
                        //     run(() -> {}, swerveDriveSubsystem),
                            run(() -> {}, shooterSubsystem).asProxy());
                }
        
        

        // Shoot for Vision Based Spinup and Aim
        // leftDriveController
        //         .getTrigger()
        //         .and(leftDriveController.getBottomThumb().negate())
        //         .and(rightDriveController.getTrigger())
        //         .whileTrue(Commands.waitUntil(() -> stoppedShootAimAndSpinup.isAtAngleAndSpunUpAndTarget())
        //                 .andThen(intakeSubsystem.shootCommand()));


        leftDriveController
                .getTrigger()
                .and(leftDriveController.getBottomThumb().negate())
                .whileTrue(deadline(
                        autoShootingCommandSpinup, run(() -> {}, lightsSubsystem).asProxy()));

        // Aim and Spinup Using Vision
        // leftDriveController
        //         .getTrigger()
        //         .and(leftDriveController.getBottomThumb())
        //         .whileTrue(deadline(
        //                 movingShootAimAndSpinup, run(() -> {}, lightsSubsystem).asProxy()));

        // Shoot for Vision Based Spinup and Aim
        leftDriveController
                .getTrigger()
                .and(rightDriveController.getTrigger())
                .whileTrue(deadline(
                        autoShootingCommand, run(() -> {}, lightsSubsystem).asProxy()));

        // Climber Down
        leftDriveController.getLeftThumb().whileTrue(climberSubsystem.setVoltage(-12));

        // Climber Up
        leftDriveController.getRightThumb().whileTrue(climberSubsystem.setVoltage(12));

        // Adjustable Shot (by default, the subwoofer speaker shot)
        LoggedReceiver topRollerSpeedTunable = Logger.tunable("/ShooterSubsystem/topTunable", ShooterSubsystem.subwooferShot.topRollerRPM);
        LoggedReceiver bottomRollerSpeedTunable = Logger.tunable("/ShooterSubsystem/bottomTunable", ShooterSubsystem.subwooferShot.bottomRollerRPM);
        LoggedReceiver pivotAngleTunable = Logger.tunable("/ShooterSubsystem/pivotTunable", ShooterSubsystem.subwooferShot.pivotAngle.getDegrees());
        LoggedReceiver isVoltageBasedTunable = Logger.tunable("/ShooterSubsystem/voltageTunable", ShooterSubsystem.subwooferShot.isAngleVoltageBased);

        leftDriveController
                .getBottomThumb()
                .and(leftDriveController.getTrigger().negate())
                // .and(rightDriveController.getBottomThumb().negate())
                .whileTrue(shooterSubsystem.shootCommand(() -> new ShooterState(
                        topRollerSpeedTunable.getDouble(),
                        bottomRollerSpeedTunable.getDouble(),
                        Rotation2d.fromDegrees(pivotAngleTunable.getDouble()),
                        isVoltageBasedTunable.getBoolean(),
                        false)));

        // leftDriveController
        //         .getBottomThumb()
        //         .and(leftDriveController.getTrigger().negate())
        //         .whileTrue(Commands.run(()-> {intakeSubsystem.setIntakeState(IntakeState.ADJUSTABLE);
        //         intakeSubsystem.topSpeedAdjustable = topRollerSpeedTunable.getDouble();
        //                 intakeSubsystem.bottomSpeedAdjustable = bottomRollerSpeedTunable.getDouble();}, intakeSubsystem));

        // Shoot for the Adjustable Shot
        leftDriveController
                .getBottomThumb()
                .and(leftDriveController.getTrigger().negate())
                .and(rightDriveController.getTrigger())
                .whileTrue(intakeSubsystem.shootCommand());

        leftDriveController
                .getLeftTopLeft()
                .whileTrue(shooterSubsystem.shootCommand(ShooterState.fromVoltages(0, 0, .6)));
        leftDriveController
                .getLeftBottomLeft()
                .whileTrue(shooterSubsystem.shootCommand(ShooterState.fromVoltages(0, 0, -.6)));

        leftDriveController.getRightTopLeft().whileTrue(climberSubsystem.overrideVoltageCommand());
        leftDriveController.getRightBottomLeft().onTrue(climberSubsystem.zeroClimberCommand());

        leftDriveController
                .getLeftBottomMiddle()
                .whileTrue(trapSubsystem.trapStateCommand(TrapState.fromVoltages(0, 0, 0.0)));
        leftDriveController
                .getLeftTopMiddle()
                .whileTrue(trapSubsystem.trapStateCommand(TrapState.fromVoltages(0, 0, 2.4)));

        leftDriveController
                .getLeftTopRight()
                .onTrue(shooterSubsystem.reengShooterAngleCommand());

        leftDriveController.getLeftBottomRight().onTrue(trapSubsystem.zeroRackPositionCommand());

        leftDriveController.getRightTopMiddle().onTrue(runOnce(() -> visionSubsystem.updatingPoseUsingVision = false));
        leftDriveController.getRightTopRight().onTrue(runOnce(() -> visionSubsystem.updatingPoseUsingVision = true));

        leftDriveController.getRightBottomMiddle().onTrue(runOnce(() -> shooterSubsystem.inPositionDisableMode = true));
        leftDriveController.getRightBottomRight().onTrue(runOnce(() -> shooterSubsystem.inPositionDisableMode = false));

        // operatorController
        //         .getLeftBumper()
        //         .and(operatorController.getRightBumper().negate())
        //         .whileTrue(parallel(shooterSubsystem.shootCommand(ShooterSubsystem.ampShot), new WaitCommand(0.5).andThen(shamperSubsystem.extendShamperCommand())));

        // operatorController
        //         .getLeftBumper()
        //         .and(operatorController.getRightBumper().negate())
        //         .whileFalse(shamperSubsystem.retractShamperCommand());

        operatorController
                .getLeftBumper()
                .and(rightDriveController.getTrigger())
                .whileTrue(intakeSubsystem.ampCommand());

        // // Trap Command
        // operatorController.getY().whileTrue(trapSubsystem.trapStateCommand(new TrapState(0, 0, 34)));

        // // Trap Amp Command
        // operatorController.getX().whileTrue(trapSubsystem.trapStateCommand(new TrapState(0, 0, 16.357)));

        // Trap Source Command
        //Command trappy = trapSubsystem.trapStateCommand(new TrapState(7, -6, 16.357));
        //Command trappyDos = trapSubsystem.trapStateCommand(new TrapState(7, -6, 16.357));
        //operatorController.getB().whileTrue(trappy.withTimeout(0.15).andThen(trappyDos.until(() -> trapSubsystem.getTopRollerCurrent() > 8)).andThen(trapSubsystem.trapStateCommand(new TrapState(7, -6, 16.357)).withTimeout(.02)));

        Command trappy = trapSubsystem.trapStateCommand(new TrapState(7, -6, 16.357));
        Command trappyDos = trapSubsystem.trapStateCommand(new TrapState(7, -6, 16.357));
        operatorController.getB().whileTrue(trappy.withTimeout(0.15).andThen(trappyDos.until(() -> trapSubsystem.getTopRollerCurrent() > 8)).andThen(trapSubsystem.trapStateCommand(new TrapState(7, -6, 16.357)).withTimeout(.02)));

        //rightDriveController.getRightThumb().whileTrue(parallel(
                //intakeSubsystem.curlCommand(), ampTransportSubsystem.ampTransportCommand(.20),trapSubsystem.runIntakeCommand(-9,9)
        //));

        

        // rightDriveController.getRightThumb().whileTrue(parallel(curling.withTimeout(0.2).andThen(curlingDos.until(() -> trapSubsystem.getTopRollerCurrent() > 8)).andThen(trapSubsystem.trapStateCommand(new TrapState(-7, 7, 0)).withTimeout(.2)),ampTransportSubsystem.ampTransportCommand(.20),trapSubsystem.runIntakeCommand(-9,9)
        // ));

        AnyContainer<Boolean> runCurler = new AnyContainer<Boolean>(false);
        Trigger runStuff = new Trigger(() -> runCurler.thing);

        

        Trigger runCurlingSetup = (rightDriveController.getRightThumb()
                .or(
                        operatorController.getDPadRight().and(operatorController.getRightBumper().negate())
                )).and(operatorController.getDPadRight().and(operatorController.getRightBumper()).negate());
                
        runCurlingSetup.whileTrue(
                        run(() -> runCurler.thing = true).until(() -> ampTransportSubsystem.hasPiece())
                        .andThen(run(() -> runCurler.thing=false).withTimeout(0.0))
                        .andThen(run(() -> runCurler.thing=true).withTimeout(0.2)))
                        .onFalse(run(() -> runCurler.thing = false));

        runStuff.whileTrue(trapSubsystem.trapStateCommand(new TrapState(-9, 9, 0)));
        runStuff.whileTrue(intakeSubsystem.curlCommand());
        runStuff.whileTrue(ampTransportSubsystem.ampTransportCommand(.20));

        //        intakeSubsystem.curlCommand(), ampTransportSubsystem.ampTransportCommand(.20),trapSubsystem.runIntakeCommand(-7.5,7.5)
        //));

        // // Trap Bottom Command (this is not zero to reduce banging. it will slowly glide down if it is below 2.5 instead
        // // of stalling)
        // operatorController.getA().whileTrue(trapSubsystem.trapStateCommand(new TrapState(0, 0, 0)));

        LoggedReceiver traptopRollerSpeedTunable = Logger.tunable("/TrapSubsystem/topTunable", 2d);
        LoggedReceiver trapbottomRollerSpeedTunable = Logger.tunable("/TrapSubsystem/bottomTunable", -2d);
        // operatorController
        //         .getDPadLeft()
        //         .whileTrue(trapSubsystem.trapStateCommand(() -> TrapState.fromVoltages(
        //                 traptopRollerSpeedTunable.getDouble(),
        //                 trapbottomRollerSpeedTunable.getDouble(),
        //                 trapSubsystem.holdingVoltage)));

        // // Trap Intake Command
        // operatorController.getDPadDown().whileTrue(trapSubsystem.runIntakeCommand(6.0, -6.0));

        // // Trap Rotate Note Up
        // operatorController
        //         .getDPadUp()
        //         .and(operatorController.getRightBumper().negate())
        //         .whileTrue(trapSubsystem.runIntakeCommand(-3.5, 3.5));
        
        // giant *quirky* trap command for people who have *quirky* fingers and keep *quirkily* pressing multiple buttons at once
        trapSubsystem.setDefaultCommand(trapSubsystem.trapStateCommand(() -> {
                TrapState state = new TrapState();
                if (trapSubsystem.getRackPosition() < 10) {
                        state = new TrapState(0, 0, 0, true);
                } else {
                        state =new TrapState(0, 0, 0.5, true);
                }

                if (operatorController.getX().getAsBoolean()) {
                        state.rack = 16.357;
                        state.isVoltageBased = false;
                }

                if (operatorController.getY().getAsBoolean()) {
                        state.rack = 34;
                        state.isVoltageBased = false;
                }

                if (operatorController.getA().getAsBoolean()) {
                        state.rack = 0;
                        state.isVoltageBased = false;
                }

                if ((operatorController.getDPadUp().getAsBoolean()
                || operatorController.getDPadUpRight().getAsBoolean()
                || operatorController.getDPadUpLeft().getAsBoolean()) && !operatorController.getRightBumper().getAsBoolean()) {
                        state.topVoltage = -3.5;
                        state.bottomVoltage = 3.5;
                }

                if ((operatorController.getDPadDown().getAsBoolean() 
                || operatorController.getDPadDownLeft().getAsBoolean() 
                || operatorController.getDPadDownRight().getAsBoolean()) && !operatorController.getRightBumper().getAsBoolean()) {
                        state.topVoltage = 6;
                        state.bottomVoltage =  -6;
                }

                if (operatorController.getDPadLeft().getAsBoolean()) {
                        state.topVoltage = traptopRollerSpeedTunable.getDouble();
                        state.bottomVoltage = trapbottomRollerSpeedTunable.getDouble();
                }

                // if (operatorController.getB().getAsBoolean()) {
                //         state.rack = 16.357;
                //         state.topVoltage = 6;
                //         state.bottomVoltage = -6;
                //         state.isVoltageBased = false;
                // }

                return state;
        }));
        // Shooter Adjustment Down
        operatorController
                .getDPadDown()
                .and(operatorController.getRightBumper())
                .onTrue(shooterSubsystem.adjustPitchCorrectionCommand(Rotation2d.fromDegrees(-.25)));

        // Shooter Adjustment Up
        operatorController
                .getDPadUp()
                .and(operatorController.getRightBumper())
                .onTrue(shooterSubsystem.adjustPitchCorrectionCommand(Rotation2d.fromDegrees(.25)));

        // Ground Intaking Indicator (Strobe Purple)
        operatorController
                .getRightTrigger()
                .and(operatorController.getLeftTrigger().negate())
                .and(() -> !intakeSubsystem.hasPieceSmoothed())
                .whileTrue(run(
                        () -> LightsSubsystemB.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystemB.purple, 0.3),
                        lightsSubsystem));

        // Trap Intaking Indicator (Flash Red and Green)
        operatorController
                .getLeftTrigger()
                .and(operatorController.getRightTrigger().negate())
                .whileTrue(repeatingSequence(
                        run(() -> LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.red), lightsSubsystem)
                                .withTimeout(.1),
                        run(
                                        () -> LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.green),
                                        lightsSubsystem)
                                .withTimeout(.1)));

        // Shooter Intaking Indicator (Flash Blue and Yellow)
        // operatorController
        //         .getLeftTrigger()
        //         .and(operatorController.getRightTrigger())
        //         .whileTrue(repeatingSequence(
        //                 run(
        //                                 () -> LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.blue),
        //                                 lightsSubsystem)
        //                         .withTimeout(.1),
        //                 run(
        //                                 () ->
        // LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.yellow),
        //                                 lightsSubsystem)
        //                         .withTimeout(.1)));

        // feeder shot (air feed)
        operatorController
                .getLeftBumper()
                .and(operatorController.getRightBumper())
                .whileTrue(Commands.either(
                        parallel(
                                swerveDriveSubsystem.cardinalCommand(
                                        new Rotation2d(-0.57), this::getDriveForwardAxis, this::getDriveStrafeAxis),
                                shooterSubsystem.shootCommand(() -> ShooterSubsystem.airFeed.plusRotation(shooterSubsystem.getPitchCorrection()))),
                        parallel(
                                swerveDriveSubsystem.cardinalCommand(
                                        new Rotation2d(Math.PI + 0.57),
                                        this::getDriveForwardAxis,
                                        this::getDriveStrafeAxis),
                                shooterSubsystem.shootCommand(() -> ShooterSubsystem.airFeed.plusRotation(shooterSubsystem.getPitchCorrection()))),
                        FieldConstants::isBlue));

        // lowfeeder shot
        operatorController
                .getLeftTrigger()
                .and(operatorController.getRightTrigger())
                .whileTrue(shooterSubsystem.shootCommand(ShooterSubsystem.groundFeedShot));

        operatorController
                .getLeftTrigger()
                .and(operatorController.getRightTrigger())
                .and(rightDriveController.getTrigger())
                .whileTrue(intakeSubsystem.shootCommand());

        operatorController.getBack().whileTrue(climberSubsystem.moveClimberUpOperator());

        operatorController
                .getStart()
                .toggleOnTrue(run(() -> LightsSubsystemB.LEDSegment.MainStrip.setRainbowAnimation(1), lightsSubsystem));

        rightDriveController.sendButtonNamesToNT();
        leftDriveController.sendButtonNamesToNT();
        operatorController.sendButtonNamesToNT();
    }

    public Command ampScoreCommand() {
        DriveToPositionCommand driveToPosition = new DriveToPositionCommand(
                swerveDriveSubsystem,
                () -> FieldConstants.getAmpPose().plus(new Transform2d(.7, -.1, new Rotation2d())),
                false);
        return Commands.parallel(driveToPosition);
    }

    public Command sourceIntakeCommand() {
        DriveToPositionCommand driveToPosition = new DriveToPositionCommand(
                swerveDriveSubsystem,
                FieldConstants.getSourcePose().plus(new Transform2d(1, 0, new Rotation2d())),
                false);
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

        DriveToPositionCommand driveToPosition =
                new DriveToPositionCommand(swerveDriveSubsystem, getNearestPose, false);
        return Commands.parallel(driveToPosition);
    }

    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand();
    }

    public Command mlIntakeCommand() {
        final double aimingFactor = .6; // up to 1 the pose converges. reduces overshoot chance if less than 1;
        final double speed = 1;
        final LinearFilter myFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
        Supplier<Rotation2d> angleOfGamepiece = () -> {
            if (visionSubsystem.getDetectorInfo().isPresent())
                return Rotation2d.fromDegrees(
                        -visionSubsystem.getDetectorInfo().get().ty() * aimingFactor);
            else return new Rotation2d();
        };

        Supplier<Rotation2d> absoluteTargetAngle = () -> Rotation2d.fromRotations(myFilter.calculate(
                angleOfGamepiece.get().plus(swerveDriveSubsystem.getRotation()).getRotations()));
        ProfiledPIDController omegaController =
                new ProfiledPIDController(2, 0, 1, new TrapezoidProfile.Constraints(4, 4));
        DoubleSupplier forward = () -> {
            if (visionSubsystem.getDetectorInfo().isPresent())
                return absoluteTargetAngle.get().getCos() * speed;
            else return 0;
        };
        DoubleSupplier strafe = () -> {
            if (visionSubsystem.getDetectorInfo().isPresent())
                return absoluteTargetAngle.get().getSin() * speed;
            else return 0;
        };

        return swerveDriveSubsystem.directionCommand(absoluteTargetAngle, forward, strafe, omegaController);
    }

    public Command mlAimCommand() {
        final double aimingFactor = .6; // up to 1 the pose converges. reduces overshoot chance if less than 1;
        final LinearFilter myFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
        Supplier<Rotation2d> angleOfGamepiece = () -> {
            if (visionSubsystem.getDetectorInfo().isPresent())
                return Rotation2d.fromDegrees(
                        -visionSubsystem.getDetectorInfo().get().ty() * aimingFactor);
            else return new Rotation2d();
        };

        Supplier<Rotation2d> absoluteTargetAngle = () -> Rotation2d.fromRotations(myFilter.calculate(
                angleOfGamepiece.get().plus(swerveDriveSubsystem.getRotation()).getRotations()));

        return swerveDriveSubsystem.directionCommandAuto(absoluteTargetAngle);
    }

    public Command autoMLIntakeTurnCommand() {
        return swerveDriveSubsystem
                .directionCommandAuto(() -> {
                    Optional<LimelightRawAngles> direction = visionSubsystem.getDetectorInfo();
                    if (direction.isPresent()) {
                        return swerveDriveSubsystem
                                .getRotation()
                                .plus(Rotation2d.fromDegrees(-direction.get().ty() * 0.7));
                    } else {
                        return swerveDriveSubsystem.getRotation();
                    }
                })
                .until(() -> intakeSubsystem.hasPieceSmoothed());
    }

    public Command mlIntakeStraightCommand() {
        SlewRateLimiter forwardSlewer = new SlewRateLimiter(8);
        SlewRateLimiter strafeSlewer = new SlewRateLimiter(8);
        final double aimingFactor = 0.8; // up to 1 the pose converges. reduces overshoot chance if less than 1;
        DoubleSupplier speed = () -> intakeSubsystem.hasPieceSmoothed() ? 0 : 1.5;
        final LinearFilter myFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
        Supplier<Rotation2d> angleOfGamepiece = () -> {
            if (visionSubsystem.getDetectorInfo().isPresent())
                return Rotation2d.fromDegrees(
                        -visionSubsystem.getDetectorInfo().get().ty() * aimingFactor);
            else return new Rotation2d();
        };

        Supplier<Rotation2d> absoluteTargetAngle = () -> Rotation2d.fromRotations(myFilter.calculate(
                angleOfGamepiece.get().plus(swerveDriveSubsystem.getRotation()).getRotations()));
        ProfiledPIDController omegaController =
                new ProfiledPIDController(4, 0, 1, new TrapezoidProfile.Constraints(6, 6));
        DoubleSupplier forward =
                () -> forwardSlewer.calculate(swerveDriveSubsystem.getRotation().getCos() * speed.getAsDouble());
        DoubleSupplier strafe =
                () -> strafeSlewer.calculate(swerveDriveSubsystem.getRotation().getSin() * speed.getAsDouble());

        return swerveDriveSubsystem
                .directionCommand(absoluteTargetAngle, forward, strafe, omegaController)
                .alongWith(runOnce(() -> {
                    ChassisSpeeds speeds = swerveDriveSubsystem.getFieldRelativeChassisSpeeds();
                    forwardSlewer.calculate(speeds.vxMetersPerSecond);
                    strafeSlewer.calculate(speeds.vyMetersPerSecond);
                }));
    }

    public double getDriveForwardAxis() {
        return (FieldConstants.isBlue() ? 1 : -1)
                * -square(deadband(leftDriveController.getYAxis().getRaw(), .15))//.15
                * Constants.SwerveConstants.maxSpeed; // );
    }

    public double getDriveStrafeAxis() {
        return (FieldConstants.isBlue() ? 1 : -1)
                * -square(deadband(leftDriveController.getXAxis().getRaw(), .15))//.15
                * Constants.SwerveConstants.maxSpeed; // );
    }

    public double getDriveRotationAxis() {
        return -cube(deadband(rightDriveController.getXAxis().getRaw(), .05))//.05
                * Constants.SwerveConstants.maxAngularVelocity
                ;
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

    /**
     * Get the spinup command primarily used in auto.
     *
     * @return         	the spinup command
     */
    public Command getSpinupCommand() {
        return new AimAndSpinupCommand(
                swerveDriveSubsystem,
                shooterSubsystem,
                lightsSubsystem,
                visionSubsystem,
                null,
                null,
                null,
                false,
                0,
                0,
                true,
                false,
                false,
                false,
                false,false);
    }

    public Command getSpinupMoveCommand() {
        return new AimAndSpinupCommand(
                swerveDriveSubsystem,
                shooterSubsystem,
                lightsSubsystem,
                visionSubsystem,
                null,
                null,
                null,
                false,
                2.0 / 16,
                0,
                true,
                false,
                false,
                false,
                false,false);
    }
}
