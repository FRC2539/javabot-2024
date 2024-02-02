package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.stream.Stream;

public class AutonomousManager {

    // Add tunables for all autonomous configuration options
    LoggedReceiver waitDuration;
    LoggedReceiver startPosition;
    LoggedReceiver gamePieces;
    LoggedReceiver shouldClimb;

    private SendableChooser<AutonomousOption> autoChooser = new SendableChooser<AutonomousOption>();

    SwerveDriveSubsystem swerveDriveSubsystem;
    LightsSubsystem lightsSubsystem;

    public AutonomousManager(RobotContainer container) {
        swerveDriveSubsystem = container.getSwerveDriveSubsystem();
        lightsSubsystem = container.getLightsSubsystem();

        // Create an event map for use in all autos
        NamedCommands.registerCommand("stop", runOnce(() -> swerveDriveSubsystem.setControl(new SwerveRequest.Idle()), swerveDriveSubsystem));
        NamedCommands.registerCommand("shoot", new Command() {});
        NamedCommands.registerCommand("intake", new Command() {});
        NamedCommands.registerCommand("mlintake", new Command() {});
        NamedCommands.registerCommand("amp", new Command() {});
        NamedCommands.registerCommand("aim", new Command() {});
        NamedCommands.registerCommand("coast", new Command() {});
        NamedCommands.registerCommand("eject", new Command() {});
        NamedCommands.registerCommand("rainbow", new Command() {});

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            Logger.log("/SwerveDriveSubsystem/targetPose", pose);
        });

        for (AutonomousOption option : AutonomousOption.values()) {
            if (option.ordinal() == 0) {
                autoChooser.setDefaultOption(option.displayName, option);
            }
            if (option.display) {
                autoChooser.addOption(option.displayName, option);
            }
        }

        Shuffleboard.getTab("Auto").add("Auto Selector", autoChooser);

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            // Logger.log("/SwerveDriveSubsystem/path", (Pose2d[]) poses.toArray());
        });
    }

    public Command getAutonomousCommand() {
        Command chosenPathCommand = new PathPlannerAuto(autoChooser.getSelected().pathName);

        var chosenWaitDuration = waitDuration.getInteger();

        if (chosenWaitDuration > 0) chosenPathCommand.beforeStarting(waitSeconds(chosenWaitDuration));

        return chosenPathCommand;
    }

    private enum AutonomousOption {
        AUTO1(
                "Speaker",
                0,
                "testAuto",
                "Test Auto",
                true
                ),

        AUTO2(
                "Amp",
                1,
                "testAuto",
                "Second Auto",
                true
                ),
        
                AUTO3(
                "Amp",
                7,
                "testAuto",
                "Seven Piece",
                true
                ),
        
        AUTO4(
                "Speaker",
                9,
                "testAuto",
                "Loop De Loop",
                true
                );

        private String pathName;
        public String startPosition;
        public int gamePieces;
        public String displayName;
        public boolean display;

        private AutonomousOption(
                String startPosition,
                int gamePieces,
                String pathName,
                String displayName,
                boolean display) {
            this.startPosition = startPosition;
            this.gamePieces = gamePieces;
            this.pathName = pathName;
            this.displayName = displayName;
            this.display = display;
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