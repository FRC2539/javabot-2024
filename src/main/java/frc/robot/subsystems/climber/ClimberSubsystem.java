package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.framework.motor.MotorIO;
import frc.lib.logging.Logger;

public class ClimberSubsystem extends SubsystemBase {
    private MotorIO pivotIO;

    private MechanismLigament2d climber;

    private final double loweredHeight = 0;
    private final double raisedHeight = 190;

    private final String stateLog = "/ClimberSubsystem/state";

    public ClimberSubsystem(MotorIO pivotIO, MechanismLigament2d climber) {
        super();
        this.pivotIO = pivotIO;

        this.climber = climber;

        setDefaultCommand(defaultCommand());
    }

    @Override
    public void periodic() {
        logClimberInformation();

        this.pivotIO.update();
    }

    private Command defaultCommand() {
        return disabledCommand();
    }

    public Command disabledCommand() {
        return startEnd(() -> {
            Logger.log(stateLog, "DISABLED");
            pivotIO.setVoltage(0);
        }, () -> {});
    }

    public Command voltageCommand(double voltage) {
        return startEnd(() -> {
            Logger.log(stateLog, "VOLTAGE");
            pivotIO.setVoltage(voltage);
        }, () -> {});
    }

    public Command zeroClimberCommand() {
        return runOnce(() -> pivotIO.zeroPosition(0));
    }

    public Command raiseCommand() {
        return voltageCommand(12).until(() -> pivotIO.getPosition() >= raisedHeight);
    }

    public Command primeCommand() {
        return voltageCommand(12).until(() -> pivotIO.getPosition() >= raisedHeight);
    }

    public Command lowerCommand() {
        return voltageCommand(-12).until(() -> pivotIO.getPosition() <= loweredHeight);
    }

    public void logClimberInformation() {
        climber.setLength(Units.inchesToMeters(22.5) + pivotIO.getPosition() / 190 * Units.inchesToMeters(21));

        Logger.log("/ClimberSubsystem/position", pivotIO.getPosition());
        Logger.log("/ClimberSubsystem/voltage", pivotIO.getVoltage());
    }
}
