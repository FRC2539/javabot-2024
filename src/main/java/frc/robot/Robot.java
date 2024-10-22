package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logging.Logger;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.lights.LightsSubsystem.LEDSegment;
import java.util.Optional;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    private Command autonomousCommand;

    public Robot() {}

    @Override
    public void robotInit() {
        // Disable default NetworkTables logging
        DataLogManager.logNetworkTables(false);

        // Begin controller inputs
        if (isReal()) {
            DriverStation.startDataLog(DataLogManager.getLog());
        }

        robotContainer = new RobotContainer(this);

        // Prevents the logging of many errors with our controllers
        DriverStation.silenceJoystickConnectionWarning(true);

        SignalLogger.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Logger.log("/Robot/Battery Voltage", RobotController.getBatteryVoltage());

        Logger.update();

        if (isSimulation()) {
            robotContainer.visionSim.update(
                    robotContainer.getSwerveDriveSubsystem().getPose());
        }
    }

    @Override
    public void autonomousInit() {

        autonomousCommand = robotContainer.getAutonomousCommand();

        LightsSubsystem.disableLEDs();
        robotContainer.getVisionSubsystem().updatingPoseUsingVision = false;

        // Schedule the chosen autonomous command
        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        robotContainer.getVisionSubsystem().updatingPoseUsingVision = true;
        LightsSubsystem.enableLEDs();

        // Prevent any autonomous code from overrunning into teleop
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {
        robotContainer.getVisionSubsystem().updatingPoseUsingVision = true;
        LightsSubsystem.enableLEDs();
    }

    @Override
    public void disabledPeriodic() {
        // // Update the autonomous command with driver station configuration
        // robotContainer.autonomousManager.update();

        if (RobotController.getBatteryVoltage() > 12.3) {
            LEDSegment.BatteryIndicator.setColor(LightsSubsystem.green.dim(.25));
        } else {
            LEDSegment.BatteryIndicator.setFadeAnimation(LightsSubsystem.green.dim(.25), 1);
        }

        if (robotContainer.getShooterSubsystem().isEncoderConnected()) {
            LEDSegment.PivotEncoderIndicator.setColor(LightsSubsystem.white.dim(.25));
        } else {
            LEDSegment.PivotEncoderIndicator.setFadeAnimation(LightsSubsystem.white.dim(.25), 1);
        }

        if (DriverStation.isDSAttached()) {
            LEDSegment.DriverstationIndicator.setColor(LightsSubsystem.orange.dim(.25));
        } else {
            LEDSegment.DriverstationIndicator.fullClear();
        }

        LEDSegment.MainStrip.setFadeAnimation(LightsSubsystem.orange, .5);

        Optional<Alliance> alliance = DriverStation.getAlliance();

        FieldConstants.isBlue = alliance.orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

        if (alliance.isPresent()) {
            if (FieldConstants.isBlue()) {
                LEDSegment.AllianceIndicator.setColor(LightsSubsystem.blue.dim(.25));
            } else {
                LEDSegment.AllianceIndicator.setColor(LightsSubsystem.red.dim(.25));
            }
        } else {
            LEDSegment.AllianceIndicator.fullClear();
        }
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationPeriodic() {
        
    }
}
