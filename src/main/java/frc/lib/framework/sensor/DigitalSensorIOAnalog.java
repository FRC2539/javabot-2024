package frc.lib.framework.sensor;

import edu.wpi.first.wpilibj.AnalogInput;

public class DigitalSensorIOAnalog implements DigitalSensorIO {
    protected boolean sensorActivated = false;
    protected boolean invert = false;

    protected AnalogInput sensor;

    /**
     * Creates a new digital sensor that looks through an analog sensor
     * @param port
     */
    public DigitalSensorIOAnalog(int port) {
        sensor = new AnalogInput(port);
    }
    
    public void update() {
        sensorActivated = sensor.getValue() > 50;
    }

    /**
     * Sets whether the sensor should be inverted
     * @param inverted true if the sensor should be inverted
     */
    public void setInvert(boolean invert) {
        this.invert = invert;
    }

    public boolean getSensor() {
        return sensorActivated ^ invert;
    }
}
