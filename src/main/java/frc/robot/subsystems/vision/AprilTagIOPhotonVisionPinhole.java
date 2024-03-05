package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.logging.Logger;
import frc.robot.Constants.FieldConstants;

public class AprilTagIOPhotonVisionPinhole implements AprilTagIO {
    Transform2d fromCameraToRobot;
    double cameraHeight;

    PhotonCamera camera;

    Supplier<Rotation2d> gyro;

    public AprilTagIOPhotonVisionPinhole(PhotonCamera camera, Transform3d fromRobotToCamera, Supplier<Rotation2d> gyro) {
        this.camera = camera;
        cameraHeight = fromRobotToCamera.getZ();
        fromCameraToRobot = new Transform2d(fromRobotToCamera.getX(), fromRobotToCamera.getY(), Rotation2d.fromRadians(fromRobotToCamera.getRotation().getZ())).inverse();
        this.gyro = gyro;
    }

    public Optional<AprilTagIOInputs> updateInputs() {
        try {
            PhotonPipelineResult results = camera.getLatestResult();
            
            PhotonTrackedTarget target = results.getBestTarget();
            if (target == null) {
                return Optional.empty();
            }

            var outputs = new AprilTagIOInputs();

            Pose3d aprilTagPosition = FieldConstants.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();

            Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
                cameraHeight, aprilTagPosition.getZ(), 
                19.0, target.getPitch(), Rotation2d.fromDegrees(-target.getYaw()), 
                gyro.get(), aprilTagPosition.toPose2d(), fromCameraToRobot);

            outputs.poseEstimate3d = new Pose3d(robotPose);
            outputs.alternatePoseEstimate3d = outputs.poseEstimate3d;

            outputs.targetDistance = PhotonUtils.getDistanceToPose(robotPose, aprilTagPosition.toPose2d());

            outputs.multitag = false; //stuff.targetsUsed.size() > 1;
            Logger.log("/VisionSubsystem/" + getName(), outputs.poseEstimate3d, true);
            Logger.log("/VisionSubsystem/" + getName() + "alternate", outputs.alternatePoseEstimate3d, true);
            Logger.log("/VisionSubsystem/" + getName() + "multitag", outputs.multitag);
            
            outputs.timestamp = results.getTimestampSeconds();
                
            return Optional.of(outputs);
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
