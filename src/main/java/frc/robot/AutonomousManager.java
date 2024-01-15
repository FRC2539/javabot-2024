package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.List;
import java.util.stream.Stream;

public class AutonomousManager {
    private static final AutonomousOption defaultAuto = AutonomousOption.OPEN_PLACE3HANDOFF;

    // Add tunables for all autonomous configuration options
    LoggedReceiver waitDuration;
    LoggedReceiver startPosition;
    LoggedReceiver gamePieces;
    LoggedReceiver shouldClimb;

    private String previousStartPosition = defaultAuto.startPosition.name();
    private int previousGamePieces = defaultAuto.gamePieces;
    private boolean previousDoesClimb = defaultAuto.doesClimb;

    private String chosenAuto = defaultAuto.pathName;

    SwerveDriveSubsystem swerveDriveSubsystem;
    LightsSubsystem lightsSubsystem;

    private boolean hasInitialized = false;

    public AutonomousManager(RobotContainer container) {
        swerveDriveSubsystem = container.getSwerveDriveSubsystem();
        lightsSubsystem = container.getLightsSubsystem();

        // Create an event map for use in all autos
        NamedCommands.registerCommand("stop", runOnce(swerveDriveSubsystem::stop, swerveDriveSubsystem));
        NamedCommands.registerCommand("flashLights", lightsSubsystem.patternCommand(LightsSubsystem.black));
    }

    public void update() {
        // Initialize in the first loop
        if (!hasInitialized) {
            initializeNetworkTables();
            hasInitialized = true;
            return;
        }

        var newStartPosition = startPosition.getString();
        var newGamePieces = gamePieces.getInteger();
        var newDoesClimb = shouldClimb.getBoolean();

        // Only update the chosen auto if a different option has been chosen
        if (previousStartPosition != newStartPosition
                || previousGamePieces != newGamePieces
                || previousDoesClimb != newDoesClimb) {
            // Match the auto based on the dashboard configuration
            List<AutonomousOption> options = Stream.of(AutonomousOption.values())
                    .filter(option -> option.startPosition.name().equals(newStartPosition)
                            && option.gamePieces == newGamePieces
                            && option.doesClimb == newDoesClimb)
                    .toList();

            if (options.size() == 1) chosenAuto = options.get(0).pathName;
            else chosenAuto = defaultAuto.pathName;

            // Determine all of the game piece options for this starting position
            long[] gamePieceOptions = Stream.of(AutonomousOption.values())
                    .filter(option ->
                            option.startPosition.name().equals(newStartPosition) && option.doesClimb == newDoesClimb)
                    .mapToLong(option -> option.gamePieces)
                    .toArray();

            Logger.log("/Autonomous/Game Piece Options", gamePieceOptions).alwaysNT();

            previousStartPosition = newStartPosition;
            previousGamePieces = (int) newGamePieces;
            previousDoesClimb = newDoesClimb;
        }
    }

    public Command getAutonomousCommand() {
        Command chosenPathCommand = new PathPlannerAuto(chosenAuto);

        var chosenWaitDuration = waitDuration.getInteger();

        if (chosenWaitDuration > 0) chosenPathCommand.beforeStarting(waitSeconds(chosenWaitDuration));

        return chosenPathCommand;
    }

    private void initializeNetworkTables() {
        waitDuration = Logger.tunable("/Autonomous/Wait Duration", 0.0);
        startPosition = Logger.tunable(
                "/Autonomous/Start Position", defaultAuto.startPosition.name()); // 0 = Left, 1 = Center, 2 = Right
        gamePieces = Logger.tunable("/Autonomous/Game Pieces", defaultAuto.gamePieces);
        shouldClimb = Logger.tunable("/Autonomous/Should Climb", defaultAuto.doesClimb);

        Logger.log("/Autonomous/Start Position Options", getStartingLocations()).alwaysNT();

        // Determine all of the game piece options for this starting position
        long[] gamePieceOptions = Stream.of(AutonomousOption.values())
                .filter(option -> option.startPosition.equals(defaultAuto.startPosition)
                        && option.doesClimb == defaultAuto.doesClimb)
                .mapToLong(option -> option.gamePieces)
                .toArray();

        Logger.log("/Autonomous/Game Piece Options", gamePieceOptions).alwaysNT();
    }

    private enum AutonomousOption {
        OPEN_PLACE3HANDOFF(
                StartingLocation.OPEN,
                3,
                false,
                "auto1"),
        STATION_PLACE1ANDCLIMB(
                StartingLocation.STATIONOPEN, 1, true, "auto1");

        private String pathName;
        public StartingLocation startPosition;
        public int gamePieces;
        public boolean doesClimb;

        private AutonomousOption(
                StartingLocation startPosition,
                int gamePieces,
                boolean doesClimb,
                String pathName) {
            this.startPosition = startPosition;
            this.gamePieces = gamePieces;
            this.doesClimb = doesClimb;
            this.pathName = pathName;
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