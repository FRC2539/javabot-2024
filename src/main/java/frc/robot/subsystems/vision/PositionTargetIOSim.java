package frc.robot.subsystems.vision;

import java.util.Optional;

public class PositionTargetIOSim implements PositionTargetIO {

    public PositionTargetIOSim() {
        
    }

    public Optional<PositionTargetIOInputs> updateInputs() {
        return Optional.empty();
    }

    public String getName() {
        return "limelight_sim";
    }
}
