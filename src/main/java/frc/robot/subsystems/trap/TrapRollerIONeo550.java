package frc.robot.subsystems.trap;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class TrapRollerIONeo550 implements TrapRollerIO {
    private CANSparkMax neo550;
    private boolean shutdown = false;

    public TrapRollerIONeo550(int port) {
        neo550 = new CANSparkMax(port, MotorType.kBrushless);

        neo550.setSmartCurrentLimit(20);
        neo550.setSecondaryCurrentLimit(25);

        neo550.burnFlash();
    }

    public void updateInputs(RollerIOInputs inputs) {
        inputs.speed = neo550.getEncoder().getVelocity();
        inputs.voltage = neo550.getAppliedOutput() * neo550.getBusVoltage();
        inputs.current = neo550.getOutputCurrent();
        inputs.motorTemperature = neo550.getMotorTemperature();

        if (inputs.motorTemperature > 60) {
            shutdown = true;
            neo550.stopMotor();
        } else if (inputs.motorTemperature < 58) {
            shutdown = false;
        }
    }

    public void setVoltage(double voltage) {
        if (shutdown) return;
        neo550.setVoltage(voltage);
    }
}
