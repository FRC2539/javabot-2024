package frc.robot.subsystems.shooter;

public class PivotIOSim implements PivotIO {
    private double pnematics1 = 0;

    public void updateInputs(PivotIOInputs inputs) {
        inputs.atTarget = true;
        inputs.currentAngle = pnematics1;
    }

    public void setAngle(double currentAngle) {
        this.pnematics1 = currentAngle;
    }
}
