package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class RollerIOFalcon implements RollerIO {
    private TalonFX talonFX;
    private final double gearRatio = 1;

    public RollerIOFalcon(int port) {
        talonFX = new TalonFX(port, "CANivore");
        talonFX.setInverted(true);
    }

    public void updateInputs(RollerIOInputs inputs) {
        inputs.speed = talonFX.getVelocity().getValue() * 60 * gearRatio; //converts rps to rpm
        inputs.voltage = talonFX.getMotorVoltage().getValue();
        inputs.current = talonFX.getStatorCurrent().getValue();
        inputs.motorTemperature = talonFX.getDeviceTemp().getValue();
    }

    public void setSpeed(double speed) {
        talonFX.setControl(new VelocityVoltage(speed / 60 /gearRatio));
    }

    public void setVoltage(double voltage) {
        talonFX.setControl(new VoltageOut(voltage));
    }
}
