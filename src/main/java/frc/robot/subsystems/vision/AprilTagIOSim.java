package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOSim implements AprilTagIO {
    public Optional<AprilTagIOInputs> updateInputs() {
        return Optional.empty();
    }

    public String getName() {
        return "Sim";
    }

    public List<PhotonTrackedTarget> updateTagsInfo() {
        return new ArrayList<PhotonTrackedTarget>();
    }
}
