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
import frc.lib.vision.LimelightHelpers;
import frc.lib.vision.LimelightHelpers.LimelightResults;
import frc.lib.vision.LimelightHelpers.Results;
import frc.robot.Constants.FieldConstants;

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
            LimelightHelpers.Results results = LimelightHelpers.getLatestResults(camera).targetingResults;

            
            
            

            var outputs = new AprilTagIOInputs();

            outputs.poseEstimate3d = results.getBotPose3d();
            outputs.alternatePoseEstimate3d = outputs.poseEstimate3d;

            outputs.targetDistance = results.botpose_avgdist;

            outputs.multitag = false; //stuff.targetsUsed.size() > 1;
            Logger.log("/VisionSubsystem/" + getName(), outputs.poseEstimate3d, true);
            Logger.log("/VisionSubsystem/" + getName() + "alternate", outputs.alternatePoseEstimate3d, true);
            Logger.log("/VisionSubsystem/" + getName() + "multitag", outputs.multitag);
            
            outputs.timestamp = results.timestamp_LIMELIGHT_publish;
                
            return Optional.of(outputs);
        } catch (Exception e) {
            return Optional.empty();
        }
    }

    public List<PhotonTrackedTarget> updateTagsInfo() {
        try {
            if (true) throw new Exception();
            Results results = LimelightHelpers.getLatestResults(camera).targetingResults;
            ArrayList<PhotonTrackedTarget> list = new ArrayList<PhotonTrackedTarget>();
            for (LimelightHelpers.LimelightTarget_Fiducial target : results.targets_Fiducials) {
                list.add(new PhotonTrackedTarget(
                    target.tx, target.ty
                    , target.ta, target.ts, (int) target.fiducialID, target.getTargetPose_CameraSpace().minus(new Pose3d()), target.getTargetPose_CameraSpace().minus(new Pose3d()), 0, null, null));
            }
            return list;
        } catch (Exception e) {
            System.out.print(e);
            return new ArrayList<PhotonTrackedTarget>();
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
