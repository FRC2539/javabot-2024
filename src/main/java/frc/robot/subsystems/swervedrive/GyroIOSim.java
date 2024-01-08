package frc.robot.subsystems.swervedrive;


public class GyroIOSim implements GyroIO {
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isActive = false;
    }
}
