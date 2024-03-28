package frc.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface AprilTagIO {
    public Pair<Optional<AprilTagIOInputs>, List<PhotonTrackedTarget>> updateInputs();

    public static class AprilTagIOInputs {
        Pose3d poseEstimate3d = new Pose3d();
        Pose3d alternatePoseEstimate3d = new Pose3d();
        double targetDistance = 0;
        double timestamp = 0;
        int numberOfTags = 0;
        boolean isValid = false;
        double targetArea = 0;
        double ambiguity = 1;
        String name = "error";
        List<PhotonTrackedTarget> targets = new ArrayList<>();
    }
}
