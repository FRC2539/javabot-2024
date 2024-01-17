package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.FieldConstants;

public class AprilTagIOPhotonVision implements AprilTagIO {
    PhotonPoseEstimator poseEstimator;

    public AprilTagIOPhotonVision(PhotonCamera camera, Transform3d fromRobotToCamera) {
        poseEstimator = new PhotonPoseEstimator(FieldConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, fromRobotToCamera);
    }

    public Optional<AprilTagIOInputs> updateInputs() {
        Optional<EstimatedRobotPose> myPose = poseEstimator.update();

        return myPose.map((stuff) -> {
            var outputs = new AprilTagIOInputs();
            outputs.poseEstimate3d = stuff.estimatedPose;
            outputs.targetDistance = Double.MAX_VALUE;
            outputs.timestamp = stuff.timestampSeconds;
            return outputs;
        });
    }
}
