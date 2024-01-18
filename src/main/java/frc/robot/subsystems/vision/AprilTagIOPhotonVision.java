package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.logging.Logger;
import frc.robot.Constants.FieldConstants;

public class AprilTagIOPhotonVision implements AprilTagIO {
    PhotonPoseEstimator poseEstimator;

    PhotonCamera camera;

    public AprilTagIOPhotonVision(PhotonCamera camera, Transform3d fromRobotToCamera) {
        poseEstimator = new PhotonPoseEstimator(FieldConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, fromRobotToCamera);
        this.camera = camera;
    }

    public Optional<AprilTagIOInputs> updateInputs() {
        Optional<EstimatedRobotPose> myPose = poseEstimator.update();

        return myPose.map((stuff) -> {
            var outputs = new AprilTagIOInputs();
            outputs.poseEstimate3d = stuff.estimatedPose;
            outputs.targetDistance = Double.MAX_VALUE;
            for (var targetUsed : stuff.targetsUsed) {
                System.out.println("cool things");
                var myDistance = targetUsed.getBestCameraToTarget().getTranslation().getNorm();
                Logger.log("/VisionSubsystem/testPoseThing"+ getName(), new Pose3d().transformBy(targetUsed.getBestCameraToTarget()), false);
                if (myDistance < outputs.targetDistance) {
                    outputs.targetDistance = myDistance;
                }
            }
            Logger.log("/VisionSubsystem/" + getName(), outputs.poseEstimate3d, true);
            outputs.timestamp = stuff.timestampSeconds;
            return outputs;
        });
    }

    public String getName() {
        return camera.getName();
    }
}
