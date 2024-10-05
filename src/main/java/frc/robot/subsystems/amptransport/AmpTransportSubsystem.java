package frc.robot.subsystems.amptransport;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpTransportSubsystem extends SubsystemBase{
    private TalonFX transportMotor;

    public AmpTransportSubsystem() {
        transportMotor = new TalonFX(10, "CANivore");
        setDefaultCommand(disabledCommand());
    }
    
    public Command ampTransportCommand(double atPercent) {
        return run(() -> {
        setAmpTransportSpeed(atPercent);
        });
    }

    public Command disabledCommand() {
        return run(() -> {
        setAmpTransportSpeed(0);
        });
    }

    public void setAmpTransportSpeed(double atPercent) {
        transportMotor.set(atPercent);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

    
