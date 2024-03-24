package frc.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOSim implements AprilTagIO {
    public Pair<Optional<AprilTagIOInputs>, List<PhotonTrackedTarget>> updateInputs() {
        return new Pair<>(Optional.empty(), new ArrayList<>());
    }

    public String getName() {
        return "Sim";
    }

    public List<PhotonTrackedTarget> updateTagsInfo() {
        return new ArrayList<PhotonTrackedTarget>();
    }
}
