package frc.robot.subsystems.amptransport;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AmpTransportSubsystem extends SubsystemBase{
    private TalonFX transportMotor;

    private DigitalInput transportSensor = new DigitalInput(0);

    public AmpTransportSubsystem() {
        transportMotor = new TalonFX(Constants.AmpTransportConstants.ampTransportMotorPort, "rio");
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

    public boolean hasPiece() {
        return transportSensor.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

    
