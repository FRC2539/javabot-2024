package frc.lib.framework;

public interface MotorIO {
    public void updateInputs(ClimberIOInputs inputs);

    public class ClimberIOInputs {
        public double position = 0;
        public double voltage = 0;
        public double current = 0;
        public double temperature = 0;
        public double positionError = 0;
        public double velocityError = 0;
    }

    public void setDutyCycle(double voltage);

    public void setVoltage(double voltage);

    public void setTargetPosition(double position);

    public void setTargetVelocity(double velocity);

    public void setTargetTorque(double torque);

    public void zeroPosition(double position);
}
