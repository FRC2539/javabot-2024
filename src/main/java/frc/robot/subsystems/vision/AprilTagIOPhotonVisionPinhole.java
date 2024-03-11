package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.logging.Logger;
import frc.robot.Constants.FieldConstants;

public class AprilTagIOPhotonVisionPinhole implements AprilTagIO {
    Transform3d fromRobotToCamera;

    PhotonCamera camera;

    Supplier<Rotation2d> gyro;

    public AprilTagIOPhotonVisionPinhole(PhotonCamera camera, Transform3d fromRobotToCamera, Supplier<Rotation2d> gyro) {
        this.camera = camera;
        this.fromRobotToCamera = fromRobotToCamera;
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

            Translation3d cameraToAprilTag = target.getBestCameraToTarget().getTranslation();

            Rotation3d tagAngle = new Rotation3d(VecBuilder.fill(1,0,0), VecBuilder.fill(cameraToAprilTag.getX(), cameraToAprilTag.getY(), cameraToAprilTag.getZ()));
            Pose2d robotPose = calculatePosition(aprilTagPosition, fromRobotToCamera, tagAngle, gyro.get());

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

    public static Pose2d calculatePosition(Pose3d aprilTagPosition, Transform3d robotToCamera, Rotation3d tagDirectionRelative, Rotation2d gyroRotation) {
        double heightDifference = aprilTagPosition.getZ() - new Pose3d().plus(robotToCamera).getZ();
        double directionHeightPerUnitDistance = new Translation3d(1, robotToCamera.getRotation().plus(tagDirectionRelative)).getZ();
        double requiredDistance = heightDifference / directionHeightPerUnitDistance;
        Translation3d robotPositionToTag = new Pose3d(new Pose2d().rotateBy(gyroRotation)).plus(robotToCamera).plus(new Transform3d(new Translation3d(requiredDistance, tagDirectionRelative), new Rotation3d())).getTranslation();

        Translation3d tagPosition = aprilTagPosition.getTranslation();

        Translation3d robotPosition = tagPosition.minus(robotPositionToTag);

        return new Pose2d(robotPosition.toTranslation2d(), gyroRotation);
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
