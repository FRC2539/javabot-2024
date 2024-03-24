package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.vision.LimelightRawAngles;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AimAndSpinupCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lights.LightsSubsystemB;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;

public class AutonomousManager {

    // Add tunables for all autonomous configuration options
    LoggedReceiver waitDuration = Logger.tunable("/Auto/waitDuration", 0d);

    private SendableChooser<AutonomousOption> autoChooser = new SendableChooser<AutonomousOption>();

    SwerveDriveSubsystem swerveDriveSubsystem;
    LightsSubsystemB lightsSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    VisionSubsystem visionSubsystem;

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
                    true,
                    true,
                    true);
            autoShootingCommand = Commands.parallel(
                    aimAndSpinupCommand,
                    waitUntil(() -> aimAndSpinupCommand.isAtAngleAndSpunUp())
                            .withTimeout(2)
                            .andThen(intakeSubsystem.shootCommand().withTimeout(0.5)));
        }
        NamedCommands.registerCommand("shoot", autoShootingCommand.asProxy());
        NamedCommands.registerCommand(
                "spinupshoot",
                parallel(container.getSpinupCommand(), intakeSubsystem.shootCommand())
                        .withTimeout(.5)); // .onlyIf(() -> intakeSubsystem.hasPiece()));
        NamedCommands.registerCommand(
                "subshoot",
                Commands.parallel(
                                shooterSubsystem
                                        .shootCommand(new ShooterState(.6, .6, Rotation2d.fromDegrees(60)))
                                        .asProxy(),
                                Commands.waitSeconds(.75)
                                        .andThen(intakeSubsystem.shootCommand().asProxy()))
                        .withTimeout(1.25));
        NamedCommands.registerCommand(
                "intake", intakeSubsystem.intakeCommand().withTimeout(2).asProxy());
        NamedCommands.registerCommand(
                "mlintake",
                swerveDriveSubsystem
                        .directionCommandAuto(() -> {
                            Optional<LimelightRawAngles> direction = visionSubsystem.getDetectorInfo();
                            if (direction.isPresent()) {
                                return swerveDriveSubsystem
                                        .getRotation()
                                        .plus(Rotation2d.fromDegrees(
                                                -direction.get().ty() * 0.7));
                            } else {
                                return swerveDriveSubsystem.getRotation();
                            }
                        })
                        .until(() -> intakeSubsystem.hasPieceSmoothed()));
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
                            System.out.println(
                                    "" + swerveDriveSubsystem.getPose().getX() + " test "
                                            + (FieldConstants.fieldLength / 2));
                            return hasPiece || pastLine;
                        })
                        .withTimeout(1)
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
                    true,
                    true,
                    true);
            autoAimCommand = Commands.parallel(aimAndSpinupCommand);
        }
        NamedCommands.registerCommand("aim", autoAimCommand);
        NamedCommands.registerCommand("spinup", container.getSpinupCommand());
        NamedCommands.registerCommand("coast", parallel());
        NamedCommands.registerCommand(
                "eject", intakeSubsystem.ejectCommand().withTimeout(2).asProxy());
        NamedCommands.registerCommand("rainbow", parallel());

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
            // Logger.log("/SwerveDriveSubsystem/path", (Pose2d[]) poses.toArray());
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
        Command chosenPathCommand = new PathPlannerAuto(autoChooser.getSelected().pathName);

        double chosenWaitDuration = 0;
        try {
            chosenWaitDuration = waitDuration.getDouble();
        } catch (Exception e) {
        }

        return chosenPathCommand.beforeStarting(waitSeconds(chosenWaitDuration));
    }

    private enum AutonomousOption {
        EASYAMP4(
                "Amp",
                4,
                "EasyAmp4",
                "Near Line (Amp)",
                true,
                "Shoots in the starting piece and then picks up and shoots the near row."),
        EASYSOURCE4(
                "Source",
                4,
                "EasySource4",
                "Near Line (Source)",
                true,
                "Shoots in the starting piece and then picks up and shoots the near row."),
        EASYCENTER4(
                "Center",
                4,
                "EasyCenter4",
                "Near Line (Center)",
                true,
                "Shoots in the starting piece and then picks up and shoots the near row."),
        SOURCE4(
                "Source",
                4,
                "Source4SCH",
                "Source Side",
                true,
                "Shoots the starting piece and then shoots the three near the source on the centerline."),

        SOURCE4A(
                "Source",
                4,
                "Source4A",
                "Source Side Conditional",
                true,
                "Shoots the starting piece and then shoots the three near the source on the centerline."),

        AMP5(
                "Amp",
                5,
                "Amp5",
                "Amp Side",
                true,
                "Scores the starting piece, the amp side near line piece, and the three pieces on the amp side centerline."),

        AMP5A(
                "Amp",
                5,
                "Amp5A",
                "Amp Side Conditional",
                true,
                "Scores the starting piece, the amp side near line piece, and the three pieces on the amp side centerline."),

        MOBILITY1(
                "Source",
                1,
                "Mobility1",
                "Mobility",
                true,
                "Shoots the starting piece and goes off to the side to get mobility."),

        CENTER5(
                "Center",
                5,
                "NewCenter5",
                "Center",
                true,
                "Shoots the starting piece, collects the next nearest piece, and scores the two most middle on the centerline."),

        LINE0("Anywhere", 0, "straightLine", "Straight Line", true, "Slowly accelerates in a straight line.");

        private String pathName;
        public String startPosition;
        public int gamePieces;
        public String displayName;
        public boolean display;
        public String description;

        private AutonomousOption(
                String startPosition,
                int gamePieces,
                String pathName,
                String displayName,
                boolean display,
                String description) {
            this.startPosition = startPosition;
            this.gamePieces = gamePieces;
            this.pathName = pathName;
            this.displayName = displayName;
            this.display = display;
            this.description = description;
        }

        private AutonomousOption(
                String startPosition, int gamePieces, String pathName, String displayName, boolean display) {
            this(startPosition, gamePieces, pathName, displayName, display, "");
        }
    }
}
