package frc.robot.subsystems.vision;

import java.util.Optional;

public interface PositionTargetIO extends CameraIO<PositionTargetIO.PositionTargetIOInputs> {
    public Optional<PositionTargetIOInputs> updateInputs();
    
    public static class PositionTargetIOInputs {
        public double yaw = 0;
        public double pitch = 0;
        public double timestamp = 0;
        public double area = 0;
    }
}
