package frc.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOSimPose implements AprilTagIO {
    public Pair<Optional<AprilTagIOInputs>, List<PhotonTrackedTarget>> updateInputs() {
        var inputs = new AprilTagIOInputs();
        inputs.poseEstimate3d = new Pose3d(new Pose2d(5, 5, new Rotation2d(Math.PI)));
        inputs.targetDistance = 5;
        inputs.timestamp = Timer.getFPGATimestamp();
        return new Pair<>(Optional.empty(), new ArrayList<>());
    }

    public String getName() {
        return "Sim";
    }

    public List<PhotonTrackedTarget> updateTagsInfo() {
        return new ArrayList<PhotonTrackedTarget>();
    }
}
