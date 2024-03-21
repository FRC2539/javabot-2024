package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.logging.Logger;
import frc.robot.Constants.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOPhotonVision implements AprilTagIO {
    PhotonPoseEstimator poseEstimator;
    PhotonPoseEstimator poseEstimatorHeight;

    PhotonCamera camera;

    public AprilTagIOPhotonVision(PhotonCamera camera, Transform3d fromRobotToCamera) {
        poseEstimator = new PhotonPoseEstimator(
                FieldConstants.aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                fromRobotToCamera);
        poseEstimatorHeight = new PhotonPoseEstimator(
                FieldConstants.aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT, camera, fromRobotToCamera);

        this.camera = camera;
    }

    public Optional<AprilTagIOInputs> updateInputs() {
        try {
            Optional<EstimatedRobotPose> myPose = poseEstimator.update();
            Optional<EstimatedRobotPose> myPoseHeight = poseEstimatorHeight.update();

            return myPose.map((stuff) -> {
                var outputs = new AprilTagIOInputs();
                outputs.poseEstimate3d = stuff.estimatedPose;
                if (myPoseHeight.isPresent()) {
                    outputs.alternatePoseEstimate3d = myPoseHeight.get().estimatedPose;
                } else {
                    outputs.alternatePoseEstimate3d = outputs.poseEstimate3d;
                }
                outputs.targetDistance = Double.MAX_VALUE;
                for (var targetUsed : stuff.targetsUsed) {
                    var myDistance =
                            targetUsed.getBestCameraToTarget().getTranslation().getNorm();
                    Logger.log(
                            "/VisionSubsystem/testPoseThing" + getName() + "" + targetUsed.getFiducialId(),
                            new Pose3d().transformBy(targetUsed.getBestCameraToTarget()),
                            false);
                    if (myDistance < outputs.targetDistance) {
                        outputs.targetDistance = myDistance;
                    }
                }
                outputs.numberOfTags = stuff.targetsUsed.size(); // stuff.targetsUsed.size() > 1;
                outputs.timestamp = stuff.timestampSeconds;

                return outputs;
            });
        } catch (Exception e) {
            return Optional.empty();
        }
    }

    public List<PhotonTrackedTarget> updateTagsInfo() {
        try {
            var results = camera.getLatestResult();
            return results.getTargets();
        } catch (Exception e) {
            System.out.print(e);
            return new ArrayList<PhotonTrackedTarget>();
        }
    }

    public String getName() {
        try {
            return camera.getName();
        } catch (Exception e) {
            System.out.print(e);
            return "error";
        }
    }
}
