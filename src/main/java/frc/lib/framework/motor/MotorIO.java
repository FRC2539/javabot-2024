package frc.lib.framework.motor;

import edu.wpi.first.math.geometry.Rotation2d;

public interface MotorIO {
    /**
     * Updates the state of the hardware abstraction. Call every loop.
     */
    public void update();

    /**
     * Request a specified duty cycle.
     *
     * @param  dutyCycle    The duty cycle value to set
     */
    public void setDutyCycle(double dutyCycle);

    /**
     * Sets the voltage of the motor.
     *
     * @param  voltage  the voltage to set
     */
    public void setVoltage(double voltage);

    /**
     * Sets the target position of the motor. For rotations is defaults to radians
     *
     * @param  position  the desired position to set
     */
    public void setTargetPosition(double position);

    /**
     * Sets the target position of the motor.
     *
     * @param  position  the desired position to set
     */
    default public void setTargetPosition(Rotation2d position) {
        setTargetPosition(position.getRadians());
    };

    /**
     * Sets the target velocity of the motor.
     *
     * @param  velocity  the desired velocity to set
     */
    public void setTargetVelocity(double velocity);

    /**
     * Sets the target current for the motor. Can be used to control torque.
     *
     * @param  current  the desired current value to set
     */
    public void setTargetCurrent(double current);

    /**
     * Sets the position of the motor to the input value. Defaults to radians.
     * <p>
     * This will not change where the motor is being told to go but will update where it thinks it is.
     *
     * @param  position  the desired position to set
     */
    public void zeroPosition(double position);

    /**
     * Sets the position of the motor to the input value.
     * <p>
     * This will not change where the motor is being told to go but will update where it thinks it is.
     *
     * @param  position  the desired position to set
     */
    default public void zeroPosition(Rotation2d position) {
        zeroPosition(position.getRadians());
    };

    /**
     * Gets the position of the motor. Defaults to radians.
     *
     * @return  the measured position
     */
    public double getPosition();

    /**
     * Gets the position of the motor.
     *
     * @return  the measured position
     */
    default public Rotation2d getPositionAsRotation2d() {
        return Rotation2d.fromRadians(getPosition());
    };

    /**
     * Gets the voltage of the motor.
     * 
     * @return  the measured voltage
     */
    public double getVoltage();

    /**
     * Gets the current through the motor.
     * 
     * @return  the measured current
     */
    public double getCurrent();

    /**
     * Gets the temperature of the motor.
     * 
     * @return  the measured temperature
     */
    public double getTemperature();

    /**
     * Gets the velocity of the motor.
     * 
     * @return the measured velocity
     */
    public double getVelocity();
}
