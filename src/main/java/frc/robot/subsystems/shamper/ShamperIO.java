
package frc.robot.subsystems.shamper;

public interface ShamperIO {

    public void updateInputs(ShamperIOInputs inputs);

    public class ShamperIOInputs {
        public double position = 0;
        public boolean atTarget = false;
        public double temperature = 0;
        public double current = 0;
        public double voltage = 0;
    }

    public void setPosition(double targetAngle);

    public void setVoltage(double voltage);

    public void zeroPosition();
}
