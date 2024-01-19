package frc.robot.subsystems.vision;

import java.util.Optional;

public interface PositionTargetIO extends CameraIO<PositionTargetIO.PositionTargetIOInputs> {
    public Optional<PositionTargetIOInputs> updateInputs();
    
    public static class PositionTargetIOInputs {
        double yaw = 0;
        double pitch = 0;
        double timestamp = 0;
    }
}
