package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.lib.logging.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.MathUtils;
import frc.lib.vision.LimelightRawAngles;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.AprilTagIO.AprilTagIOInputs;
import frc.robot.subsystems.vision.PositionTargetIO.PositionTargetIOInputs;

public class VisionSubsystem extends SubsystemBase {
    public static double translationStdDevCoefficient = 1;
    public static double rotationStdDevCoefficient = 1.2;

    private final double multitagTranslationStdDevCoefficient = 0.1;
    private final double multitagRotationStdDevCoefficient = 0.5;

    private AprilTagIO left;
    private AprilTagIO right;
    private PositionTargetIO limelight;
    private PositionTargetIO limelightApriltag;

    private Optional<AprilTagIOInputs> leftInputs = Optional.empty();
    private Optional<AprilTagIOInputs> rightInputs = Optional.empty();
    private Optional<PositionTargetIOInputs> limelightInputs = Optional.empty();
    private Optional<PositionTargetIOInputs> limelightApriltagInputs = Optional.empty();

    public List<PhotonTrackedTarget> leftTargets = new ArrayList<>();
    public List<PhotonTrackedTarget> rightTargets = new ArrayList<>();

    private SwerveDriveSubsystem consumer;

    public VisionSubsystem(SwerveDriveSubsystem consumer, AprilTagIO left, AprilTagIO right, PositionTargetIO limelight, PositionTargetIO limelightApriltag) {
        this.left = left;
        this.right = right;
        this.limelight = limelight;
        this.limelightApriltag = limelightApriltag;

        this.consumer = consumer;
    }

    public void periodic() {
        leftInputs = left.updateInputs();
        rightInputs = right.updateInputs();
        limelightInputs = limelight.updateInputs();
        limelightApriltagInputs = limelightApriltag.updateInputs();

        leftTargets = left.updateTagsInfo();
        rightTargets = right.updateTagsInfo();

        addVisionPoseEstimate(leftInputs);
        addVisionPoseEstimate(rightInputs);
    }

    public static Optional<PhotonTrackedTarget> getTagInfo(List<PhotonTrackedTarget> targets, int tagID) {
        for (var target : targets) {
            if (target.getFiducialId() == tagID) {
                return Optional.of(target);
            }
        }
        return Optional.empty();
    }

    public Optional<LimelightRawAngles> getDetectorInfo() {
        return limelightInputs.map((inputs) -> new LimelightRawAngles(inputs.yaw, inputs.pitch, inputs.area));
    }

    public Optional<LimelightRawAngles> getTagAngleInfo() {
        return limelightApriltagInputs.map((inputs) -> new LimelightRawAngles(inputs.yaw, inputs.pitch, inputs.area));
    }

    private void addVisionPoseEstimate(Optional<AprilTagIOInputs> inputs) {
        inputs.ifPresent((t) -> {
            if (t.multitag) {
                addVisionPoseEstimateWithBackupMultitag(t.poseEstimate3d, t.alternatePoseEstimate3d, t.targetDistance, t.timestamp);
            } {
                addVisionPoseEstimateWithBackup(t.poseEstimate3d, t.alternatePoseEstimate3d, t.targetDistance, t.timestamp);
            }
        });
    }

    private void addVisionPoseEstimate(Pose3d estimate, double distance, double timestamp) {
        if (!isWithinField(estimate)) return;

        // TODO: acutlally calculate

        consumer.addVisionMeasurement(
            estimate.toPose2d(), timestamp, calculateVisionStdDevs(distance));
    }

    private void addVisionPoseEstimateTwoTags(Pose3d estimate, double distance, double timestamp) {
        if (!isWithinField(estimate)) return;

        // TODO: acutlally calculate

        consumer.addVisionMeasurement(
            estimate.toPose2d(), timestamp, calculateMultitagVisionStdDevs(distance));
    }

    private void addVisionPoseEstimateWithBackup(Pose3d estimate, Pose3d backup, double distance, double timestamp) {
        if (!isWithinField(estimate)) {
            addVisionPoseEstimate(backup, distance, timestamp);
        } else {
            addVisionPoseEstimate(estimate, distance, timestamp);
        }
    }

    private void addVisionPoseEstimateWithBackupMultitag(Pose3d estimate, Pose3d backup, double distance, double timestamp) {
        if (!isWithinField(estimate)) {
            addVisionPoseEstimateTwoTags(backup, distance, timestamp);
        } else {
            addVisionPoseEstimateTwoTags(estimate, distance, timestamp);
        }
    }

    private static boolean isWithinField(Pose3d pose) {
        boolean isWithinField = MathUtils.isInRange(pose.getY(), -2.5, FieldConstants.fieldWidth + 2.5)
                && MathUtils.isInRange(pose.getX(), -2.5, FieldConstants.fieldLength + 2.5)
                && MathUtils.isInRange(pose.getZ(), -.5, .5);

        // boolean isNearRobot = consumer
        //                 .getPose()
        //                 .getTranslation()
        //                 .getDistance(pose.getTranslation().toTranslation2d())
                // < 1.4;

        return isWithinField;// && isNearRobot;
    }


    public Matrix<N3, N1> calculateVisionStdDevs(double distance) {
        var translationStdDev = translationStdDevCoefficient * distance;
        var rotationStdDev = rotationStdDevCoefficient * distance;

        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }

    public Matrix<N3, N1> calculateMultitagVisionStdDevs(double distance) {
        var translationStdDev = multitagTranslationStdDevCoefficient * distance;
        var rotationStdDev = multitagRotationStdDevCoefficient * distance;

        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }

    private Rotation2d lastSpeakerAngle = new Rotation2d(0);
    public Rotation2d getSpeakerAngle(Pose2d currentPose) {
        Optional<PhotonTrackedTarget> speakerTag  = VisionSubsystem.getTagInfo(leftTargets, FieldConstants.getSpeakerTag());
        if (speakerTag.isPresent() && false) {
            // TODO: This is not right
            double distance = getSpeakerDistance(currentPose);
            Transform2d transformToCamera = new Transform2d(
                VisionConstants.robotToLeftCamera.getTranslation().toTranslation2d(), 
                new Rotation2d(VisionConstants.robotToLeftCamera.getRotation().getZ()));
            Transform2d transformToGoal = transformToCamera.plus(new Transform2d(distance, 0, Rotation2d.fromDegrees(speakerTag.get().getYaw())));

            // Gets that final translation to goal and
            lastSpeakerAngle = currentPose.getRotation().plus(transformToGoal.getTranslation().getAngle());
        } else {
            lastSpeakerAngle = FieldConstants.getSpeakerPose().getTranslation().minus(currentPose.getTranslation()).getAngle();
        }
        Logger.log("/VisionSubsystem/LastSpeakerAngle", lastSpeakerAngle.getRadians());
        return lastSpeakerAngle;
    }
    public double getSpeakerDistance(Pose2d currentPose) {
        Optional<PhotonTrackedTarget> speakerTag  = VisionSubsystem.getTagInfo(leftTargets, FieldConstants.getSpeakerTag());
        double distance;
        if (speakerTag.isPresent() && false) {
            // TODO: This is not right
            distance = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.robotToLeftCamera.getZ(), Units.inchesToMeters(57.75), 
                Units.degreesToRadians(19), Units.degreesToRadians(speakerTag.get().getPitch()));
        } else {
            distance = FieldConstants.getSpeakerPose().getTranslation().minus(currentPose.getTranslation()).getNorm();
        }
        Logger.log("/VisionSubsystem/LastSpeakerDistance", distance);
        return distance;
    }
}
