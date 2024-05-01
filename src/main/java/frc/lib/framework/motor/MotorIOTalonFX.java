package frc.lib.framework.motor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;

public class MotorIOTalonFX implements MotorIO {
    protected TalonFX motor;

    protected double position = 0;
    protected double voltage = 0;
    protected double current = 0;
    protected double temperature = 0;
    protected double velocity = 0;

    protected StatusSignal<Double> positionSignal;
    protected StatusSignal<Double> voltageSignal;
    protected StatusSignal<Double> currentSignal;
    protected StatusSignal<Double> temperatureSignal;
    protected StatusSignal<Double> velocitySignal;

    protected DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    protected VoltageOut voltageOut = new VoltageOut(0);
    protected TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0);
    protected PositionVoltage positionVoltage = new PositionVoltage(0);
    protected VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    /**
     * Constructs a new instance of the <code>MotorIOTalonFX</code> class. 
     * 
     * This also includes all default configurations for motors, such as default current limits.
     *
     * @param port The CAN bus port number to which the Talon FX is connected.
     * @param canbus The CAN bus name to which the Talon FX is connected.
     */
    public MotorIOTalonFX(int port, String canbus) {
        motor = new TalonFX(port, canbus);
        positionSignal = motor.getPosition();
        voltageSignal = motor.getMotorVoltage();
        currentSignal = motor.getStatorCurrent();
        temperatureSignal = motor.getDeviceTemp();
        velocitySignal = motor.getVelocity();

        TalonFXConfigurator configurator = motor.getConfigurator();

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        configurator.refresh(currentLimitsConfigs);
        currentLimitsConfigs.StatorCurrentLimit = 80;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 90;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        configurator.apply(currentLimitsConfigs);

        TorqueCurrentConfigs torqueCurrentConfigs = new TorqueCurrentConfigs();
        configurator.refresh(torqueCurrentConfigs);
        torqueCurrentConfigs.PeakForwardTorqueCurrent = 80;
        torqueCurrentConfigs.PeakReverseTorqueCurrent = -80;
        configurator.apply(torqueCurrentConfigs);

        zeroPosition(0);
    }

    
    public void update() {
        position = Units.rotationsToRadians(positionSignal.refresh().getValueAsDouble());
        voltage = voltageSignal.refresh().getValueAsDouble();
        current = currentSignal.refresh().getValueAsDouble();
        temperature = temperatureSignal.refresh().getValueAsDouble();
        velocity = Units.rotationsToRadians(velocitySignal.refresh().getValueAsDouble());
    }

    public void setDutyCycle(double dutyCycle) {
        motor.setControl(dutyCycleOut.withOutput(dutyCycle));
    }

    public void setVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    public void setTargetPosition(double position) {
        motor.setControl(positionVoltage.withPosition(Units.radiansToRotations(position)));
    }

    public void setTargetVelocity(double velocity) {
        motor.setControl(velocityVoltage.withVelocity(Units.radiansToRotations(velocity)));
    }

    public void setTargetCurrent(double torque) {
        motor.setControl(torqueCurrentFOC.withOutput(torque));
    }

    public void zeroPosition(double position) {
        motor.setPosition(Units.radiansToRotations(position));
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
