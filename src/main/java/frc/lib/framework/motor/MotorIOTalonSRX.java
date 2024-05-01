package frc.lib.framework.motor;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class MotorIOTalonSRX implements MotorIO {
    protected TalonSRX motor;
    protected TalonSRXConfiguration config;

    protected double position = 0;
    protected double voltage = 0;
    protected double current = 0;
    protected double temperature = 0;
    protected double velocity = 0;



    /**
     * Constructs a new instance of the <code>MotorIOTalonFX</code> class. 
     * 
     * This also includes all default configurations for motors, such as default current limits.
     *
     * @param port The CAN bus port number to which the Talon FX is connected.
     */
    public MotorIOTalonSRX(int port) {
        motor = new TalonSRX(port);

        config = new TalonSRXConfiguration();


        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 1));

        motor.configAllSettings(config);

        zeroPosition(0);

        motor.setNeutralMode(NeutralMode.Brake);
    }

    
    public void update() {
        position = motor.getSelectedSensorPosition() / 4096.0 * 2.0 * Math.PI;
        voltage = motor.getMotorOutputVoltage();
        current = motor.getStatorCurrent();
        temperature = motor.getTemperature();
        velocity = motor.getSelectedSensorVelocity() / 4096.0 * 2.0 * Math.PI * 10.0;
    }

    public void setDutyCycle(double dutyCycle) {
        motor.enableVoltageCompensation(false);
        motor.set(TalonSRXControlMode.PercentOutput, dutyCycle);
    }

    public void setVoltage(double voltage) {
        motor.enableVoltageCompensation(true);
        motor.set(TalonSRXControlMode.PercentOutput, voltage * 12);
    }

    public void setTargetPosition(double position) {
        motor.enableVoltageCompensation(true);
        motor.set(TalonSRXControlMode.Position, position * 4096.0 / (2.0 * Math.PI));
    }

    public void setTargetVelocity(double velocity) {
        motor.enableVoltageCompensation(true);
        motor.set(TalonSRXControlMode.Velocity, velocity * 4096.0 / (2.0 * Math.PI) / 10.0);
    }

    public void setTargetCurrent(double torque) {
        motor.enableVoltageCompensation(true);
        motor.set(TalonSRXControlMode.Current, torque);
    }

    public void zeroPosition(double position) {
        motor.setSelectedSensorPosition(position * 4096.0 / (2.0 * Math.PI));
    }


    public double getPosition() {
        return position;
    }

    public double getVoltage() {
        return voltage;
    }

    public double getCurrent() {
        return current;
    }

    public double getTemperature() {
        return temperature;
    }

    public double getVelocity() {
        return velocity;
    }
}