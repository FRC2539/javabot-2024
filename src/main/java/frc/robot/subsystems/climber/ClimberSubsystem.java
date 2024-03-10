package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.logging.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ClimberSubsystem extends SubsystemBase {
    private ClimberIO pivotIO;

    private ClimberIOInputs climberVoltage = new ClimberIOInputs();

    private double voltage = 0;
    private boolean overrideMode = false;

    private final double lowerLimit = 0;
    private final double upperLimit = 190; //315
    private final double operatorLimit = 200;

    public ClimberSubsystem(ClimberIO pivotIO) {
        this.pivotIO = pivotIO;

        setDefaultCommand(disabledCommand());
    }
    

    public void periodic() {
        logClimberInformation();

        this.pivotIO.updateInputs(climberVoltage);

        var tempVoltage = voltage;
        if (voltage < 0 && climberVoltage.currentPosition < lowerLimit && !overrideMode) {
            tempVoltage = 0;
        }
        
        if (voltage > 0 && climberVoltage.currentPosition > upperLimit) {
            tempVoltage = 0;
        }

        this.pivotIO.setVoltage(tempVoltage);
    }

    public Command disabledCommand() {
        return run(() -> {
            voltage = 0;
        });
    }

    public Command setVoltage(double voltage) {
        return run(() -> {this.voltage = voltage;});
    }

    public Command overrideVoltageCommand() {
        return runEnd(() -> {
            voltage = -2;
            overrideMode = true;
        }, () -> overrideMode = false
            );
    }

    public Command zeroClimberCommand() {
        return runOnce(() -> pivotIO.setPosition(0));
    }

    public Command moveClimberUpOperator() {
        return setVoltage(12).until(() -> climberVoltage.currentPosition >= operatorLimit);
    }
    
    public void logClimberInformation() {
        Logger.log("/ClimberSubsystem/position", climberVoltage.currentPosition);
        Logger.log("/ClimberSubsystem/voltageCommanded", voltage);
        Logger.log("/ClimberSubsystem/voltage", climberVoltage.currentVoltage);
    }
}
