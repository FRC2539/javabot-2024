package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils.AnyContainer;
import frc.lib.vision.LimelightRawAngles;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AimAndSpinupCommand;
import frc.robot.commands.IntakeAssistCommandComplexAuto;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lights.LightsSubsystemB;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class AutonomousManager {

    // Add tunables for all autonomous configuration options
    LoggedReceiver waitDuration = Logger.tunable("/Auto/waitDuration", 0d);

    private SendableChooser<AutonomousOption> autoChooser = new SendableChooser<AutonomousOption>();

    SwerveDriveSubsystem swerveDriveSubsystem;
    LightsSubsystemB lightsSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    VisionSubsystem visionSubsystem;

    public static int trajectoryNumber = 0;
    public static ChoreoTrajectory currentChoreo = null;
    public static Timer currentAutoTime = new Timer();

    public AutonomousManager(RobotContainer container) {
        swerveDriveSubsystem = container.getSwerveDriveSubsystem();
        lightsSubsystem = container.getLightsSubsystem();
        intakeSubsystem = container.getIntakeSubsystem();
        shooterSubsystem = container.getShooterSubsystem();
        visionSubsystem = container.getVisionSubsystem();

        // Create an event map for use in all autos

        // It appears that for any default commands to run, the commands need to be registered as proxies. Howevery,
        // anything that uses swerve cannot have the swerve proxied out.
        NamedCommands.registerCommand(
                "stop",
                runOnce(() -> swerveDriveSubsystem.setControl(new SwerveRequest.Idle()), swerveDriveSubsystem)
                        .asProxy());

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
                    true);
            autoShootingCommand = Commands.deadline(
                    Commands.waitSeconds(0.5)
                            .andThen(waitUntil(() -> aimAndSpinupCommand.isAtAngleAndSpunUpAndTarget())
                                    .withTimeout(2.0))
                            .andThen(intakeSubsystem.shootCommand().asProxy().withTimeout(0.4)),
                    aimAndSpinupCommand,
                    run(() -> {}, swerveDriveSubsystem),
                    run(() -> {}, shooterSubsystem).asProxy());
        }
        NamedCommands.registerCommand("shoot", autoShootingCommand);
        // Command aimedShootCommand;
        // {
        //     AimAndSpinupCommand aimAndSpinupCommand = new AimAndSpinupCommand(
        //             swerveDriveSubsystem,
        //             shooterSubsystem,
        //             lightsSubsystem,
        //             visionSubsystem,
        //             () -> 0,
        //             () -> 0,
        //             () -> 0,
        //             false,
        //             0,
        //             0,
        //             true,
        //             true,
        //             true,
        //             true,
        //             true);
        //     aimedShootCommand = Commands.deadline(
        //                     Commands.waitSeconds(0.5)
        //                             .andThen(waitUntil(() -> aimAndSpinupCommand.isAtAngleAndSpunUpAndTarget())
        //                                     .withTimeout(2.0))
        //                             .andThen(intakeSubsystem
        //                                     .shootCommand()
        //                                     .withTimeout(0.4)
        //                                     .asProxy()),
        //                     aimAndSpinupCommand,
        //                     run(() -> {}, swerveDriveSubsystem),
        //                     run(() -> {}, shooterSubsystem).asProxy())
        //             .finallyDo(() -> visionSubsystem.updatingPoseUsingVision = false)
        //             .beforeStarting(() -> visionSubsystem.updatingPoseUsingVision = false);
        // }
        NamedCommands.registerCommand("aimedshoot", autoShootingCommand); // .onlyIf(() -> intakeSubsystem.hasPiece()));
        NamedCommands.registerCommand(
                "subshoot",
                Commands.parallel(
                                shooterSubsystem
                                        .shootCommand(new ShooterState(.6, .6, Rotation2d.fromDegrees(62)))
                                        .asProxy(),
                                Commands.waitSeconds(.5)
                                        .andThen(intakeSubsystem.shootCommand().asProxy()))
                        .withTimeout(1.0));
        NamedCommands.registerCommand("intake", intakeSubsystem.intakeCommand().asProxy());
        @SuppressWarnings("unused")
        LinearFilter lowPassIQR = LinearFilter.movingAverage(20);
        IntakeAssistCommandComplexAuto intakeAssistCommandComplex =
                new IntakeAssistCommandComplexAuto(swerveDriveSubsystem, visionSubsystem, lightsSubsystem);

        swerveDriveSubsystem.autoStrafeOverrideSupplier = (Double x) -> intakeAssistCommandComplex.transformStrafe(x);

        @SuppressWarnings("unused")
        Command intakeAssistCommandTurn = swerveDriveSubsystem.directionCommandAutoVelocity(
                () -> {
                    Optional<LimelightRawAngles> direction = visionSubsystem.getDetectorInfo();
                    if (direction.isPresent()) {
                        return new Rotation2d(swerveDriveSubsystem
                                .getPoseAtTimestamp(direction.get().timestamp())
                                .getRotation()
                                .plus(Rotation2d.fromDegrees(-direction.get().ty()))
                                .getRadians());
                    } else {
                        return swerveDriveSubsystem.getRotation();
                    }
                },
                new PIDController(5, 0, 0.1));

        NamedCommands.registerCommand(
                "mlintake", intakeAssistCommandComplex.until(() -> intakeSubsystem.hasPieceSmoothed()));

        NamedCommands.registerCommand(
                "mlintakedrive",
                container
                        .mlIntakeStraightCommand()
                        .until(() -> {
                            boolean hasPiece = intakeSubsystem.hasPieceSmoothed();
                            boolean pastLine =
                                    false; // FieldConstants.isBlue() == (swerveDriveSubsystem.getPose().getX() >
                            // ((FieldConstants.fieldLength / 2) - 0));
                            Logger.log("Auto/pastLine", pastLine);
                        //     System.out.println(
                        //             "" + swerveDriveSubsystem.getPose().getX() + " test "
                        //                     + (FieldConstants.fieldLength / 2));
                            return hasPiece || pastLine;
                        })
                        .alongWith(intakeSubsystem.intakeCommand().asProxy()));
        NamedCommands.registerCommand("amp", parallel());
        Command autoAimCommand;
        {
            AimAndSpinupCommand aimAndSpinupCommand = new AimAndSpinupCommand(
                    swerveDriveSubsystem,
                    shooterSubsystem,
                    lightsSubsystem,
                    visionSubsystem,
                    () -> 0,
                    () -> 0,
                    () -> 0,
                    true,
                    0,
                    0,
                    true,
                    true,
                    true,
                    false,
                    true);
            autoAimCommand = aimAndSpinupCommand.asProxy();
        }
        NamedCommands.registerCommand(
                "search",
                Commands.run(() -> swerveDriveSubsystem.setControl(swerveDriveSubsystem
                                .openLoop
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(FieldConstants.isBlue() ? 2 : -2)))
                        .withTimeout(1.5)
                        .andThen(Commands.waitSeconds(10))
                        .until(() -> visionSubsystem.getDetectorInfo().isPresent()));
        NamedCommands.registerCommand("aim", autoAimCommand);
        NamedCommands.registerCommand("spinup", container.getSpinupCommand().asProxy());
        NamedCommands.registerCommand(
                "spinupshoot",
                parallel(
                                container.getSpinupCommand().asProxy(),
                                intakeSubsystem.shootCommand().asProxy())
                        .withTimeout(.5));
        NamedCommands.registerCommand(
                "spinupmov", container.getSpinupMoveCommand().asProxy());
        NamedCommands.registerCommand(
                "spinupshootmov",
                parallel(
                                container.getSpinupMoveCommand().asProxy(),
                                intakeSubsystem.shootCommand().asProxy())
                        .withTimeout(.5));
        NamedCommands.registerCommand("coast", parallel());
        NamedCommands.registerCommand(
                "eject", intakeSubsystem.ejectCommand().withTimeout(2).asProxy());
        NamedCommands.registerCommand("rainbow", parallel());

        registerChoreoCommands();

        // Run sussy paths conditionally
        registerConditionalPaths();

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            Logger.log("/SwerveDriveSubsystem/targetPose", pose);
        });

        for (AutonomousOption option : AutonomousOption.values()) {
            if (option.ordinal() == 0) {
                autoChooser.setDefaultOption(option.displayName, option);

                Logger.log("/Auto/startPosition", option.startPosition);
                Logger.log("/Auto/gamePieces", option.gamePieces);
                Logger.log("/Auto/description", option.description);
            }
            if (option.display) {
                autoChooser.addOption(option.displayName, option);
            }
        }

        autoChooser.onChange((option) -> {
            Logger.log("/Auto/startPosition", option.startPosition);
            Logger.log("/Auto/gamePieces", option.gamePieces);
            Logger.log("/Auto/description", option.description);
        });

        SmartDashboard.putData("AutoChooser", autoChooser);

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            // System.out.println("Path length: " + poses.size());
            if (poses.size() > 0) {
                trajectoryNumber += 1;
                currentAutoTime.restart();
            }
        });
    }

    private Command choreoTrajectoryPublisher(String name) {
        return new Command() {
            private List<ChoreoTrajectory> trajectories = Choreo.getTrajectoryGroup(name);
            @Override
            public void initialize() {
            }

            @Override
            public void execute() {
                try {
                        var cirCor = trajectories.get(trajectoryNumber);
                        if (!FieldConstants.isBlue()) {
                                cirCor = cirCor.flipped();
                        }
                        currentChoreo = cirCor;
                } catch (IndexOutOfBoundsException e) {
                        currentChoreo = null;
                }
                Logger.log("PathPlanner/trajectoryNumber", trajectoryNumber);
                if (currentChoreo != null) {
                        Logger.log("PathPlanner/trajectoryEndPose", currentChoreo.getFinalState().getPose());
                }
                Logger.log("PathPlanner/trajectoryTime", currentAutoTime.get());
                Logger.log("PathPlanner/isSpinningUp", spinningUp);
            }

            @Override
            public void end(boolean interrupted) {
                currentChoreo = null;
            }
        };
    }

    private boolean spinningUp = false;

    private void registerChoreoCommands() {
        // docs detail on how these work
        // Shooting is composed of three named commands.
        // * a spinup command
        // * an aim command
        // * and a shoot command
        // the spinup can be run at any time and runs continuously until you either try running another spinup command or you shoot
        // the aim commands can only be run at a stop point and will run until you run a shoot command
        // the shoot commands can be run at any time and shoots a piece and then shuts off all aim and spinup commands
        // the wait for shoot command is an aim command that does nothing basically
        NamedCommands.registerCommand("c_spinup_x", scheduleWrapper(spinupPredictive()));
        NamedCommands.registerCommand("c_spinup_0.5", scheduleWrapper(spinupPredictive(0.5)));
        NamedCommands.registerCommand("c_spinup_1.0", scheduleWrapper(spinupPredictive(1.0)));
        NamedCommands.registerCommand("c_spinup_1.5", scheduleWrapper(spinupPredictive(1.5)));

        NamedCommands.registerCommand("c_spinup_setpoint_1", scheduleWrapper(spinupSetpoint(new ShooterState(0.6,0.6, Rotation2d.fromDegrees(52)))));

        NamedCommands.registerCommand("c_spinup_vision", scheduleWrapper(spinupVision()));

        NamedCommands.registerCommand("c_spinup_pose", scheduleWrapper(spinupPose()));

        NamedCommands.registerCommand("c_aim_vision", new Command() {});
        NamedCommands.registerCommand("c_aim_pose", new Command() {});
        NamedCommands.registerCommand("c_aim_move", new Command() {});
        NamedCommands.registerCommand("c_wait_for_shoot", waitUntil(() -> !spinningUp));

        NamedCommands.registerCommand("c_shoot", scheduleWrapper(shootCommand()));


        NamedCommands.registerCommand("c_intake", scheduleWrapper(new Command() {}));
        NamedCommands.registerCommand("c_note_strafe", scheduleWrapper(new Command() {}));
        NamedCommands.registerCommand("c_strafe_intake", scheduleWrapper(new Command() {})); // just does the above two together
    }

    private Command scheduleWrapper(Command command) {
            return runOnce(() -> {
                // System.out.println("starting schedule!!!!!!!!!!!!!!");
                command.asProxy().onlyWhile(DriverStation::isAutonomousEnabled).schedule();
                
            });
    }

    private Command spinupWrapper(Command command) {
        return (command.beforeStarting(() -> {
                // System.out.println("starting spinup!!!!!!!!!!!!!!")
        })).beforeStarting(() -> spinningUp = true).until(() -> !spinningUp);
    }

    private Command spinupSupplier(DoubleSupplier distance) {
        return spinupWrapper(shooterSubsystem.shootCommand(distance));
    }

    private Command spinupPredictive() {
        AnyContainer<OptionalDouble> distanceToSupplier = new AnyContainer<>(OptionalDouble.empty());
        return spinupSupplier(() -> {
            if (distanceToSupplier.thing.isEmpty()) {
                ChoreoTrajectoryState future = currentChoreo.getFinalState();
                Pose2d shootingPosition = future.getPose();
                double distanceToSpeaker = shootingPosition.getTranslation().getDistance(FieldConstants.getSpeakerPose().getTranslation());
                distanceToSupplier.thing = OptionalDouble.of(distanceToSpeaker);
            }
            return distanceToSupplier.thing.getAsDouble();
        });
    }

    private Command spinupPredictive(double timeInFuture) {
        // TODO: Predictive spinup
        AnyContainer<OptionalDouble> distanceToSupplier = new AnyContainer<>(OptionalDouble.empty());
        
        return spinupSupplier(() -> {
                if (distanceToSupplier.thing.isEmpty()) {
                ChoreoTrajectoryState future = currentChoreo.sample(timeInFuture + currentAutoTime.get(), FieldConstants.isBlue());
                        Pose2d shootingPosition = future.getPose();
                        double distanceToSpeaker = shootingPosition.getTranslation().getDistance(FieldConstants.getSpeakerPose().getTranslation());
                        distanceToSupplier.thing = OptionalDouble.of(distanceToSpeaker);
                }
                return distanceToSupplier.thing.getAsDouble();
        });
    }

    private Command spinupSetpoint(ShooterState state) {
        return spinupWrapper(shooterSubsystem.shootCommand(state));
    }

    private Command spinupVision() {
        return spinupSupplier(() -> { 
                OptionalDouble distance =  visionSubsystem.getSpeakerDistanceFromVision(swerveDriveSubsystem.getPose());
                if (distance.isPresent()) {
                        return distance.getAsDouble();
                } else {
                        return visionSubsystem.getSpeakerDistanceFromPose(swerveDriveSubsystem.getPose());
                }
        });
    }

    private Command spinupPose() {
        return spinupSupplier(() -> { 
                return visionSubsystem.getSpeakerDistanceFromPose(swerveDriveSubsystem.getPose());
        });
    }

    private Command shootCommand() {
        // Runs the intake to shoot
        return intakeSubsystem.shootCommand().withTimeout(0.5).finallyDo(() -> 
        {spinningUp = false;
        //System.out.println("finished shoot command?");
        });
    }

    private Command pathFromFile(String name) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name));
    }

    private void registerConditionalPaths() {
        registerShootConditionalPath("1b-1s-2b", "1b-1s", "1b-2bml", "1b-2bml");
        registerShootConditionalPath("2b-1s-3b", "2b-1s", "1s-3bml", "2b-3bml");

        NamedCommands.registerCommand(
                "3b-2s-3b",
                Commands.either(
                        Commands.sequence(
                                pathFromFile("3b-2s"), NamedCommands.getCommand("shoot"), pathFromFile("2s-3b")),
                        Commands.runOnce(() -> {}),
                        () -> intakeSubsystem.hasPieceSmoothed()));

        registerShootConditionalPath("5b-3s-4b", "-3s", "-4b", "5b-4b");
        registerShootConditionalPath("4b-3s-3b", "-3s", "3s-3b", "4b-3b");
        registerShootConditionalPath("3b-3s-5b", "3b-3s", "3s-5b", "3b-5b");
    }

    private void registerShootConditionalPath(String commandName, String path1, String path2, String altPath) {
        NamedCommands.registerCommand(
                commandName,
                Commands.either(
                        Commands.sequence(
                                pathFromFile(path1),
                                NamedCommands.getCommand("shoot"),
                                pathFromFile(path2),
                                NamedCommands.getCommand("mlintakedrive")),
                        Commands.sequence(pathFromFile(altPath), NamedCommands.getCommand("mlintakedrive")),
                        () -> intakeSubsystem.hasPieceSmoothed()));
    }

    public Command getAutonomousCommand() {
        AutonomousOption option = autoChooser.getSelected();
        Command chosenPathCommand;
        if (option.pathName.equals("sysidtranslation")) {
            chosenPathCommand =
                    swerveDriveSubsystem.sysIdRoutineCommand(swerveDriveSubsystem.sysIdSwerveTranslationRoutine);
        } else if (option.pathName.equals("sysidrotation")) {
            chosenPathCommand =
                    swerveDriveSubsystem.sysIdRoutineCommand(swerveDriveSubsystem.sysIdSwerveRotationRoutine);
        } else if (option.pathName.equals("sysidsteergains")) {
            chosenPathCommand =
                    swerveDriveSubsystem.sysIdRoutineCommand(swerveDriveSubsystem.sysIdSwerveSteerGainsRoutine);
        } else {
            chosenPathCommand = new PathPlannerAuto(option.pathName);
        }

        double chosenWaitDuration = 0;
        try {
            chosenWaitDuration = waitDuration.getDouble();
        } catch (Exception e) {
        }

        if (option.isChoreo) {
                return chosenPathCommand.beforeStarting(runOnce(() -> {trajectoryNumber = -1;})
                .andThen(waitSeconds(chosenWaitDuration))).deadlineWith(choreoTrajectoryPublisher(option.pathName));
        } else {
                return chosenPathCommand.beforeStarting(waitSeconds(chosenWaitDuration));
        }
    }

    private enum AutonomousOption {
        // Does
        CENTER5A(
                "Center",
                5,
                "NewCenter5",
                "Center (Second Note) DEPRECATED",
                true,
                "Preload + Nearline + Second Note on Centerline"),
        CENTER5B(
                "Center",
                5,
                "NewCenterCenter5Loopy",
                "Center (Center Note) DEPRECATED",
                true,
                "Preload + Newarline + Center Note on Centerline"),
        CENTER5BS(
                "Center",
                5,
                "NewCenterCenter5Search",
                "Center (Center Note Search) DEPRECATED",
                true,
                "Preload + Newarline + Center Note on Centerline"),
        CENTER5C(
                "Center",
                5,
                "NewCenterPole5",
                "Center (Pole First) (Second Note) DEPRECATED",
                true,
                "Preload + Nearline + Second Note on Centerline"),
        // EASYAMP4(
        //         "Amp",
        //         4,
        //         "EasyAmp4",
        //         "Near Line (Amp)",
        //         false,
        //         "Shoots in the starting piece and then picks up and shoots the near row."),
        // EASYSOURCE4(
        //         "Source",
        //         4,
        //         "EasySource4",
        //         "Near Line (Source)",
        //         false,
        //         "Shoots in the starting piece and then picks up and shoots the near row."),
        // EASYCENTER4(
        //         "Center",
        //         4,
        //         "EasyCenter4",
        //         "Near Line (Center)",
        //         false,
        //         "Shoots in the starting piece and then picks up and shoots the near row."),
        SOURCE4(
                "Source",
                4,
                "NewSource4",
                "Source Side DEPRECATED",
                true,
                "Preload + First on Centerline + Second on Centerline + Grab Third on Centerline."),
        SOURCE4B(
                "Source",
                4,
                "NewSourceCenter4",
                "Source Side (Second Piece) DEPRECATED",
                true,
                "Preload + Second on Centerline + Third on Centerline."),

        // SOURCE4A(
        //         "Source",
        //         4,
        //         "Source4A",
        //         "Source Side Conditional",
        //         false,
        //         "Shoots the starting piece and then shoots the three near the source on the centerline."),

        AMP5("Amp", 5, "NewAmp5", "Amp Side DEPRECATED", true, "Preload + First on Centerline + Second on Centerline."),
        AMP5B(
                "Amp",
                5,
                "NewAmp5Sketch",
                "Amp Side (Second) DEPRECATED",
                true,
                "Preload + First on Centerline + Second on Centerline."),
        SIXPIECE("Center", 6, "SixPiece", "Six Piece", true, "", true),
        SYS_ID_TRANSLATION("Arbitrary", 0, "sysidtranslation", "sysid translation", true, ""),
        SYS_ID_ROTATION("Arbitrary", 0, "sysidrotation", "sysid rotation", true, ""),
        SYS_ID_STEER_GAINS("Arbitrary", 0, "sysidsteergains", "sysid steer gains", true, "");
        // AMP5A(
        //         "Amp",
        //         5,
        //         "Amp5A",
        //         "Amp Side Conditional",
        //         false,
        //         "Scores the starting piece, the amp side near line piece, and the three pieces on the amp side
        // centerline."),

        // MOBILITY1(
        //         "Source",
        //         1,
        //         "Mobility1",
        //         "Mobility",
        //         false,
        //         "Shoots the starting piece and goes off to the side to get mobility."),

        // LINE0("Anywhere", 0, "straightLine", "Straight Line", false, "Slowly accelerates in a straight line.");

        private String pathName;
        public String startPosition;
        public int gamePieces;
        public String displayName;
        public boolean display;
        public String description;
        public boolean isChoreo;

        private AutonomousOption(
                String startPosition,
                int gamePieces,
                String pathName,
                String displayName,
                boolean display,
                String description, 
                boolean isChoreo) {
            this.startPosition = startPosition;
            this.gamePieces = gamePieces;
            this.pathName = pathName;
            this.displayName = displayName;
            this.display = display;
            this.description = description;
            this.isChoreo = isChoreo;
        }

        private AutonomousOption(
                String startPosition, int gamePieces, String pathName, String displayName, boolean display) {
            this(startPosition, gamePieces, pathName, displayName, display, "");
        }

        private AutonomousOption(
                String startPosition, int gamePieces, String pathName, String displayName, boolean display, String description) {
            this(startPosition, gamePieces, pathName, displayName, display, description, false);
        }
    }
}
