package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

import java.util.Optional;
import java.util.stream.Stream;

public class AutonomousManager {

    // Add tunables for all autonomous configuration options
    LoggedReceiver waitDuration = Logger.tunable("/Auto/waitDuration", 0d);

    private SendableChooser<AutonomousOption> autoChooser = new SendableChooser<AutonomousOption>();

    SwerveDriveSubsystem swerveDriveSubsystem;
    LightsSubsystem lightsSubsystem;
    IntakeSubsystem intakeSubsystem;

    public AutonomousManager(RobotContainer container) {
        swerveDriveSubsystem = container.getSwerveDriveSubsystem();
        lightsSubsystem = container.getLightsSubsystem();
        intakeSubsystem = container.getIntakeSubsystem();

        // Create an event map for use in all autos

        //It appears that for any default commands to run, the commands need to be registered as proxies. Howevery, anything that uses swerve cannot have the swerve proxied out.
        NamedCommands.registerCommand("stop", runOnce(() -> swerveDriveSubsystem.setControl(new SwerveRequest.Idle()), swerveDriveSubsystem).asProxy());
        NamedCommands.registerCommand("shoot", container.stoppedShootAndAimCommand(Optional.of(1d)).onlyIf(() -> intakeSubsystem.hasPiece()));
        NamedCommands.registerCommand("intake", intakeSubsystem.intakeCommand().withTimeout(2).asProxy());
        NamedCommands.registerCommand("mlintake", parallel());
        NamedCommands.registerCommand("amp", parallel());
        NamedCommands.registerCommand("aim", container.movingAimCommandAuto());
        NamedCommands.registerCommand("coast", parallel());
        NamedCommands.registerCommand("eject", intakeSubsystem.ejectCommand().withTimeout(2).asProxy());
        NamedCommands.registerCommand("rainbow", parallel());

        //Run sussy paths conditionally
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

    public void registerConditionalPaths() {
        NamedCommands.registerCommand("1b-1s-2b", Commands.either(
            Commands.sequence(pathFromFile("-1s"), NamedCommands.getCommand("shoot"), pathFromFile("-2b")),
            pathFromFile("1b-2b"), 
            () -> intakeSubsystem.hasPiece()));
        
        NamedCommands.registerCommand("2b-1s-3b", Commands.either(
            Commands.sequence(pathFromFile("-1s"), NamedCommands.getCommand("shoot"), pathFromFile("1s-3b")),
            pathFromFile("2b-3b"), 
            () -> intakeSubsystem.hasPiece()));
        
        NamedCommands.registerCommand("3b-2s-3b", Commands.either(
            Commands.sequence(pathFromFile("3b-2s"), NamedCommands.getCommand("shoot"), pathFromFile("2s-3b")),
            Commands.runOnce(() -> {}), 
            () -> intakeSubsystem.hasPiece()));
        
        NamedCommands.registerCommand("5b-3s-4b", Commands.either(
            Commands.sequence(pathFromFile("-3s"), NamedCommands.getCommand("shoot"), pathFromFile("-4b")),
            pathFromFile("5b-4b"), 
            () -> intakeSubsystem.hasPiece()));
        
        NamedCommands.registerCommand("4b-3s-3b", Commands.either(
            Commands.sequence(pathFromFile("-3s"), NamedCommands.getCommand("shoot"), pathFromFile("3s-3b")),
            pathFromFile("4b-3b"), 
            () -> intakeSubsystem.hasPiece()));
        
        NamedCommands.registerCommand("3b-3s-5b", Commands.either(
            Commands.sequence(pathFromFile("3b-3s"), NamedCommands.getCommand("shoot"), pathFromFile("3s-5b")),
            pathFromFile("3b-5b"), 
            () -> intakeSubsystem.hasPiece()));
    }

    public Command getAutonomousCommand() {
        Command chosenPathCommand = new PathPlannerAuto(autoChooser.getSelected().pathName);

        double chosenWaitDuration = 0;
        try {
            chosenWaitDuration = waitDuration.getDouble();
        } catch (Exception e) {}

        return chosenPathCommand.beforeStarting(waitSeconds(chosenWaitDuration));
    }

    private enum AutonomousOption {
        EASY4(
                "Amp",
                4,
                "Easy4",
                "Near Line",
                true,
                "Shoots in the starting piece and then picks up and shoots the near row."
                ),

        SOURCE4(
                "Source",
                4,
                "Source4",
                "Source Side",
                true,
                "Shoots the starting piece and then shoots the three near the source on the centerline."
                ),
        
        SOURCE4A(
                "Source",
                4,
                "Source4A",
                "Source Side Conditional",
                true,
                "Shoots the starting piece and then shoots the three near the source on the centerline."
                ),
        
        AMP5(
                "Amp",
                5,
                "Amp5",
                "Amp Side",
                true,
                "Scores the starting piece, the amp side near line piece, and the three pieces on the amp side centerline."
                ),
        
        AMP5A(
                "Amp",
                5,
                "Amp5A",
                "Amp Side Conditional",
                true,
                "Scores the starting piece, the amp side near line piece, and the three pieces on the amp side centerline."
                ),
        
        MOBILITY1(
            "Source",
            1,
            "Mobility1",
            "Mobility",
            true,
            "Shoots the starting piece and goes off to the side to get mobility."
        ),

        LINE0(
            "Anywhere",
            0,
            "straightLine",
            "Straight Line",
            true,
            "Slowly accelerates in a straight line."
        );

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
                String startPosition,
                int gamePieces,
                String pathName,
                String displayName,
                boolean display) {
            this(startPosition, gamePieces, pathName, displayName, display, "");
        }
    }

    public static String[] getAutonomousOptionNames() {
        return Stream.of(AutonomousOption.values()).map(AutonomousOption::name).toArray(String[]::new);
    }
}