package frc.robot.subsystems.vision;

import java.util.Optional;

public interface CameraIO<T> {
    public Optional<T> updateInputs();
}
