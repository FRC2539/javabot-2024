package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagIO extends CameraIO<AprilTagIO.AprilTagIOInputs> {
    public Optional<AprilTagIOInputs> updateInputs();
    
    public static class AprilTagIOInputs {
        Pose3d poseEstimate3d = new Pose3d();
        Pose3d alternatePoseEstimate3d = new Pose3d();
        double targetDistance = 0;
        double timestamp = 0;
        boolean multitag = false;
    }

    public List<PhotonTrackedTarget> updateTagsInfo();
}
