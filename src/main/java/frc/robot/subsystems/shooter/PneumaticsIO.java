package frc.robot.subsystems.shooter;

public interface PneumaticsIO {

    public void updateInputs(PneumaticsIOInputs inputs);

    public class PneumaticsIOInputs {
        public boolean forward1 = false;
        public boolean forward2 = false;
    }

    public void setPosition(boolean forward1, boolean forward2);
    
    public int getPosition();
}
