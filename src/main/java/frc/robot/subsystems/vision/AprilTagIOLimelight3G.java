package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.vision.LimelightHelpers;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOLimelight3G implements AprilTagIO {
    Transform3d fromRobotToCamera;

    String camera;

    Supplier<Rotation2d> gyro;

    public AprilTagIOLimelight3G(String name, Transform3d fromRobotToCamera, Supplier<Rotation2d> gyro) {
        this.camera = name;
        this.fromRobotToCamera = fromRobotToCamera;
        this.gyro = gyro;
    }

    public Optional<AprilTagIOInputs> updateInputs() {
        try {
            var results = LimelightHelpers.getBotPoseEstimate_wpiBlue(camera);

            var outputs = new AprilTagIOInputs();

            if (results.pose.getX() == 0.0) return Optional.empty();

            if (results.rawFiducials.length == 0) return Optional.empty();

            outputs.poseEstimate3d = new Pose3d(results.pose);
            outputs.alternatePoseEstimate3d = new Pose3d(results.pose);
            outputs.ambiguity = results.rawFiducials[0].ambiguity;
            outputs.isValid = true;
            outputs.numberOfTags = results.tagCount;
            outputs.targetArea = results.avgTagArea;
            outputs.targetDistance = results.avgTagDist;
            outputs.timestamp = results.timestampSeconds;

            outputs.targets = new ArrayList<>();

            for (LimelightHelpers.RawFiducial target : results.rawFiducials) {
                outputs.targets.add(new PhotonTrackedTarget(
                        target.txnc,
                        target.tync,
                        target.ta,
                        0,
                        target.id,
                        new Transform3d(),
                        new Transform3d(),
                        target.ambiguity,
                        null,
                        null));
            }

            return Optional.of(outputs);
        } catch (Exception e) {
            return Optional.empty();
        }
    }

    public String getName() {
        try {
            return camera;
        } catch (Exception e) {
            System.out.print(e);
            return "error";
        }
    }
}
