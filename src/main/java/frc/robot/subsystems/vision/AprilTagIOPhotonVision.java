package frc.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
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

    boolean invertPitchAndYaw;

    public AprilTagIOPhotonVision(PhotonCamera camera, Transform3d fromRobotToCamera) {
        this(camera, fromRobotToCamera, false);
    }

    public AprilTagIOPhotonVision(PhotonCamera camera, Transform3d fromRobotToCamera, boolean invertPitchAndYaw) {
        poseEstimator = new PhotonPoseEstimator(
                FieldConstants.aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                fromRobotToCamera);
        poseEstimatorHeight = new PhotonPoseEstimator(
                FieldConstants.aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT, camera, fromRobotToCamera);

        this.camera = camera;
        this.invertPitchAndYaw = invertPitchAndYaw;
    }

    public Pair<Optional<AprilTagIOInputs>, List<PhotonTrackedTarget>> updateInputs() {
        try {
            Optional<EstimatedRobotPose> myPose = poseEstimator.update();
            Optional<EstimatedRobotPose> myPoseHeight = poseEstimatorHeight.update();

            return new Pair<>(
                    myPose.map((stuff) -> {
                        var outputs = new AprilTagIOInputs();
                        outputs.poseEstimate3d = stuff.estimatedPose;
                        if (myPoseHeight.isPresent()) {
                            outputs.alternatePoseEstimate3d = myPoseHeight.get().estimatedPose;
                        } else {
                            outputs.alternatePoseEstimate3d = outputs.poseEstimate3d;
                        }
                        outputs.targetDistance = Double.MAX_VALUE;
                        for (var targetUsed : stuff.targetsUsed) {
                            var myDistance = targetUsed
                                    .getBestCameraToTarget()
                                    .getTranslation()
                                    .getNorm();
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

                        outputs.targets = updateTagsInfo();

                        return outputs;
                    }),
                    updateTagsInfo());
        } catch (Exception e) {
            System.out.print(e);
            return new Pair<>(Optional.empty(), new ArrayList<>());
        }
    }

    private List<PhotonTrackedTarget> updateTagsInfo() {
        try {
            var results = camera.getLatestResult();
            List<PhotonTrackedTarget> outputList = new ArrayList<>();
            for (var target : results.getTargets()) {
                outputList.add(new PhotonTrackedTarget(
                        target.getYaw() * (invertPitchAndYaw ? -1 : 1),
                        target.getPitch(),// * (invertPitchAndYaw ? -1 : 1),
                        target.getArea(),
                        target.getSkew(),
                        target.getFiducialId(),
                        target.getBestCameraToTarget(),
                        target.getAlternateCameraToTarget(),
                        target.getPoseAmbiguity(),
                        target.getMinAreaRectCorners(),
                        target.getDetectedCorners()));
            }
            return outputList;
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
