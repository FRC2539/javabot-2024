package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import frc.lib.logging.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.MathUtils;
import frc.lib.vision.LimelightRawAngles;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.AprilTagIO.AprilTagIOInputs;
import frc.robot.subsystems.vision.PositionTargetIO.PositionTargetIOInputs;

public class VisionSubsystem extends SubsystemBase {
    private final double translationStdDevCoefficient = 0.35;
    private final double rotationStdDevCoefficient = 1;

    private AprilTagIO left;
    private AprilTagIO right;
    private PositionTargetIO limelight;

    private Optional<AprilTagIOInputs> leftInputs = Optional.empty();
    private Optional<AprilTagIOInputs> rightInputs = Optional.empty();
    private Optional<PositionTargetIOInputs> limelightInputs = Optional.empty();

    public List<PhotonTrackedTarget> leftTargets = new ArrayList<>();
    public List<PhotonTrackedTarget> rightTargets = new ArrayList<>();

    private CommandSwerveDrivetrain consumer;

    public VisionSubsystem(CommandSwerveDrivetrain consumer, AprilTagIO left, AprilTagIO right, PositionTargetIO limelight) {
        this.left = left;
        this.right = right;
        this.limelight = limelight;

        this.consumer = consumer;
    }

    public void periodic() {
        leftInputs = left.updateInputs();
        rightInputs = right.updateInputs();
        limelightInputs = limelight.updateInputs();

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

    private void addVisionPoseEstimate(Optional<AprilTagIOInputs> inputs) {
        inputs.ifPresent((t) -> addVisionPoseEstimateWithBackup(t.poseEstimate3d, t.alternatePoseEstimate3d, t.targetDistance, t.timestamp));
    }

    private void addVisionPoseEstimate(Pose3d estimate, double distance, double timestamp) {
        if (!isWithinField(estimate)) return;

        // TODO: acutlally calculate

        consumer.addVisionMeasurement(
            estimate.toPose2d(), timestamp); //calculateVisionStdDevs(distance));
    }

    private void addVisionPoseEstimateWithBackup(Pose3d estimate, Pose3d backup, double distance, double timestamp) {
        if (!isWithinField(estimate)) {
            addVisionPoseEstimate(backup, distance, timestamp);
        } else {
            addVisionPoseEstimate(estimate, distance, timestamp);
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

    private Matrix<N3, N1> calculateVisionStdDevs(double distance) {
        var translationStdDev = translationStdDevCoefficient * Math.pow(distance, 2);
        var rotationStdDev = rotationStdDevCoefficient * Math.pow(distance, 2);

        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }
}
