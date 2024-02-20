package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.interpolation.InterpolatableDouble;
import frc.lib.interpolation.InterpolatingMap;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.subsystems.shooter.PivotIO.PivotIOInputs;
import frc.robot.subsystems.shooter.RollerIO.RollerIOInputs;

public class ClimberSubsystem extends SubsystemBase {
    private final double shooterSpeedTolerance = 0.1;
    private final double shooterAngleTolerance = 0.01;

    private ClimberIO pivotIO;

    private double currentDistance;

    private ClimberIOInputs climberVoltage = new ClimberIOInputs();

    private double voltage = 0;

    public ClimberSubsystem(ClimberIO pivotIO) {
        this.pivotIO = pivotIO;

        setDefaultCommand(disabledCommand());
    }
    

    public void periodic() {
        logClimberInformation();

        this.pivotIO.updateInputs(climberVoltage);

        this.pivotIO.setVoltage(voltage);
    }

    public Command disabledCommand() {
        return run(() -> {
            voltage = 0;
        });
    }

    public Command setVoltage(double voltage) {
        return run(() -> {this.voltage = voltage;});
    }
    
    public void logClimberInformation() {
        
    }
}
