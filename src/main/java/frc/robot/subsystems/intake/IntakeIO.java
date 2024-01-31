package frc.robot.subsystems.intake;

public interface IntakeIO {
    
    public void updateInputs(IntakeIOInputs inputs);

    public class IntakeIOInputs {
        public double rollerSpeed = 0;
        public double rollerVoltage = 0;
        public double rollerCurrent = 0;
        public double rollerTemperature = 0;

        public double chamberSpeed = 0;
        public double chamberVoltage = 0;
        public double chamberCurrent = 0;
        public double chamberTemperature = 0;

        public boolean rollerSensor = false;
        public boolean chamberSensor = false;
    }

    /**
     * This sets the speed of the intake roller as a duty cycle from -1 to 1.
     * @param speed
     */
    public void setRollerSpeed(double speed);

    /**
     * This sets the speed of the interior belt transportation system as a duty cycle from -1 to 1.
     * @param speed
     */
    public void setChamberSpeed(double speed);
}
