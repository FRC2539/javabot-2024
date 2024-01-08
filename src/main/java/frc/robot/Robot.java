package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logging.Logger;
import frc.lib.swerve.CTREConfigs;

public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs = new CTREConfigs();

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
    }

    @Override
    public void autonomousInit() {

        autonomousCommand = robotContainer.getAutonomousCommand();

        // Schedule the chosen autonomous command
        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {

        // Prevent any autonomous code from overrunning into teleop
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // Update the autonomous command with driver station configuration
        robotContainer.autonomousManager.update();
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
