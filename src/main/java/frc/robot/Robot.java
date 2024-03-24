package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logging.Logger;
import frc.robot.subsystems.lights.LightsSubsystemB;
import frc.robot.subsystems.lights.LightsSubsystemB.LEDSegment;

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
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Logger.log("/Robot/Battery Voltage", RobotController.getBatteryVoltage());

        Logger.update();

        robotContainer.visionSim.update(robotContainer.getSwerveDriveSubsystem().getPose());
    }

    @Override
    public void autonomousInit() {

        autonomousCommand = robotContainer.getAutonomousCommand();

        LightsSubsystemB.disableLEDs();
        robotContainer.getVisionSubsystem().usingVision = false;

        // Schedule the chosen autonomous command
        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        robotContainer.getVisionSubsystem().usingVision = true;
        LightsSubsystemB.enableLEDs();

        // Prevent any autonomous code from overrunning into teleop
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {
        robotContainer.getVisionSubsystem().usingVision = true;
        LightsSubsystemB.enableLEDs();
    }

    @Override
    public void disabledPeriodic() {
        // // Update the autonomous command with driver station configuration
        // robotContainer.autonomousManager.update();

        if (RobotController.getBatteryVoltage() > 12.3) {
            LEDSegment.BatteryIndicator.setColor(LightsSubsystemB.green.dim(.25));
        } else {
            LEDSegment.BatteryIndicator.setFadeAnimation(LightsSubsystemB.green.dim(.25), 1);
        }

        // if (robotContainer.getShooterSubsystem().) {
        //     LEDSegment.BatteryIndicator.setColor(LightsSubsystemB.green.dim(.25));
        // } else {
        //     LEDSegment.BatteryIndicator.setFadeAnimation(LightsSubsystemB.green.dim(.25),1);
        // }

        if (DriverStation.isDSAttached()) {
            LEDSegment.DriverStationIndicator.setColor(LightsSubsystemB.orange.dim(.25));
        } else {
            LEDSegment.DriverStationIndicator.fullClear();
        }

        LEDSegment.MainStrip.setFadeAnimation(LightsSubsystemB.orange, .5);
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
