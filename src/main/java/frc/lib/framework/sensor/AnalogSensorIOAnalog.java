package frc.lib.framework.sensor;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogSensorIOAnalog implements AnalogSensorIO {
    protected int sensorValue = 0;
    protected boolean invert = false;

    protected AnalogInput sensor;

    /**
     * Creates a new analog sensor that looks through an analog sensor
     * @param port
     */
    public AnalogSensorIOAnalog(int port) {
        sensor = new AnalogInput(port);
    }
    
    public void update() {
        sensorValue = sensor.getValue();
    }

    public double getSensor() {
        return sensorValue;
    }
}
