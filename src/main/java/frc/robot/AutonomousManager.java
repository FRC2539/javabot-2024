package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
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
        NamedCommands.registerCommand("stop", runOnce(() -> swerveDriveSubsystem.setControl(new SwerveRequest.Idle()), swerveDriveSubsystem));
        NamedCommands.registerCommand("shoot", parallel());
        NamedCommands.registerCommand("intake", intakeSubsystem.intakeCommand().withTimeout(.5));
        NamedCommands.registerCommand("mlintake", parallel());
        NamedCommands.registerCommand("amp", parallel());
        NamedCommands.registerCommand("aim", parallel());
        NamedCommands.registerCommand("coast", parallel());
        NamedCommands.registerCommand("eject", intakeSubsystem.ejectCommand().withTimeout(2));
        NamedCommands.registerCommand("rainbow", parallel());

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

    public Command getAutonomousCommand() {
        Command chosenPathCommand = new PathPlannerAuto(autoChooser.getSelected().pathName);

        double chosenWaitDuration = 0;
        try {
            chosenWaitDuration = waitDuration.getDouble();
        } catch (Exception e) {}

        return chosenPathCommand.beforeStarting(waitSeconds(chosenWaitDuration));
    }

    private enum AutonomousOption {
        AUTO1(
                "Amp",
                0,
                "testAuto",
                "Test Auto",
                true,
                "Goes in circles in front of the amp 6 times."
                ),

        AUTO2(
                "Amp",
                4,
                "testAuto1",
                "Near Line",
                true,
                "Picks up all three near pieces and shoots."
                ),
        
        AUTO3(
                "Amp",
                3,
                "testAuto2",
                "Midline",
                true,
                "Picks up the piece nearest to the amp and then one on the midline."
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

    public static String[] getStartingLocations() {
        return Stream.of(StartingLocation.values()).map(StartingLocation::name).toArray(String[]::new);
    }

    public static String[] getAutonomousOptionNames() {
        return Stream.of(AutonomousOption.values()).map(AutonomousOption::name).toArray(String[]::new);
    }

    private enum StartingLocation {
        OPEN,
        STATIONOPEN,
        STATIONCABLE,
        CABLE
    }
}