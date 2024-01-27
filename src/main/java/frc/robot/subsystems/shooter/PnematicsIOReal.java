package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PnematicsIOReal implements PneumaticsIO {
    private boolean forward1 = false;
    private boolean forward2 = false;

    private DoubleSolenoid doubleSolenoid1;
    private DoubleSolenoid doubleSolenoid2;

    public PnematicsIOReal() {
        doubleSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
        doubleSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
    }

    public void updateInputs(PneumaticsIOInputs inputs) {
        inputs.forward1 = forward1;
        inputs.forward2 = forward2;
    }

    public void setPosition(boolean forward1, boolean forward2) {
        this.forward1 = forward1;
        this.forward2 = forward2;

        if (forward1) {
            doubleSolenoid1.set(DoubleSolenoid.Value.kForward);
        }
        else {
            doubleSolenoid1.set(DoubleSolenoid.Value.kReverse);
        }

        if (forward2) {
            doubleSolenoid2.set(DoubleSolenoid.Value.kForward);
        }
        else {
            doubleSolenoid2.set(DoubleSolenoid.Value.kReverse);
        }
    }
}
