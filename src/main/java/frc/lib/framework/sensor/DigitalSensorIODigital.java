package frc.lib.framework.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalSensorIODigital {
    protected boolean sensorActivated = false;
    protected boolean invert = false;

    protected DigitalInput sensor;

    public DigitalSensorIODigital(int port) {
        sensor = new DigitalInput(port);
    }
    
    public void update() {
        sensorActivated = sensor.get();
    }

    /**
     * Sets whether the sensor should be inverted
     * @param inverted true if the sensor should be inverted
     */
    public void setInvert(boolean inverted) {
        this.invert = inverted;
    }

    public boolean getSensor() {
        return sensorActivated ^ invert;
    }
}
