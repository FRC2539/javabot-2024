package frc.robot.subsystems.trap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class TrapRollerIOFalcon implements TrapRollerIO {
    private CANSparkMax neo550;
    private boolean shutdown = false;

    public TrapRollerIOFalcon(int port) {
        neo550 = new CANSparkMax(port, MotorType.kBrushless);
        
        neo550.setSmartCurrentLimit(20);
        neo550.setSecondaryCurrentLimit(25);

        neo550.burnFlash();
    }

    public void updateInputs(RollerIOInputs inputs) {
        inputs.speed = neo550.getEncoder().getVelocity(); //converts rps to rpm
        inputs.voltage = neo550.getAppliedOutput() * neo550.getBusVoltage();
        inputs.current = neo550.getOutputCurrent();
        inputs.motorTemperature = neo550.getMotorTemperature();

        if (inputs.motorTemperature > 40) {
            shutdown = true;
            neo550.stopMotor();
        } else if (inputs.motorTemperature < 30) {
            shutdown = false;
        }
    }

    public void setVoltage(double voltage) {
        if (shutdown) return;
        neo550.setVoltage(voltage);
    }
}
