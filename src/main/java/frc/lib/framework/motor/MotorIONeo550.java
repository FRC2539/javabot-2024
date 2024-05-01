package frc.lib.framework.motor;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

public class MotorIONeo550 implements MotorIO {
    protected CANSparkMax motor;
    protected boolean shutdown = false;
    protected SparkPIDController pidController;

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
     * @param canbus The CAN bus name to which the Talon FX is connected.
     */
    public MotorIONeo550(int port) {
        motor = new CANSparkMax(port, MotorType.kBrushless);

        motor.setSmartCurrentLimit(20);
        motor.setSecondaryCurrentLimit(25);

        motor.getEncoder().setPosition(0);

        motor.setIdleMode(IdleMode.kBrake);

        pidController = motor.getPIDController();

        motor.burnFlash();
    }

    
    public void update() {
        position = Units.rotationsToRadians(motor.getEncoder().getPosition());
        voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        current = motor.getOutputCurrent();
        temperature = motor.getMotorTemperature();
        velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
    }

    public void setDutyCycle(double dutyCycle) {
        motor.set(dutyCycle);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setTargetPosition(double position) {
        pidController.setReference(Units.radiansToRotations(position), ControlType.kPosition);
    }

    public void setTargetVelocity(double velocity) {
        pidController.setReference(Units.radiansToRotations(velocity), ControlType.kVelocity);
    }

    public void setTargetCurrent(double torque) {
        pidController.setReference(Units.radiansToRotations(torque), ControlType.kCurrent);
    }

    public void zeroPosition(double position) {
        motor.getEncoder().setPosition(position);
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
