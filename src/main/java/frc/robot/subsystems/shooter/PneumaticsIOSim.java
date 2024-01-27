package frc.robot.subsystems.shooter;

public class PneumaticsIOSim implements PneumaticsIO {
    private boolean pnematics1 = false;
    private boolean pnematics2 = false;

    public void updateInputs(PneumaticsIOInputs inputs) {
        inputs.forward1 = pnematics1;
        inputs.forward2= pnematics2;
    }

    public void setPosition(boolean pnematics1, boolean pnematics2) {
        this.pnematics1 = pnematics1;
        this.pnematics2 = pnematics2;
    }
}
