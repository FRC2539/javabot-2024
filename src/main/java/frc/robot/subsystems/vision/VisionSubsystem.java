package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.lib.vision.LimelightRawAngles;
import frc.lib.vision.PinholeModel3D;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.AprilTagIO.AprilTagIOInputs;
import frc.robot.subsystems.vision.PositionTargetIO.PositionTargetIOInputs;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    public static double translationStdDevCoefficient = 1;
    public static double rotationStdDevCoefficient = 1.2;

    private final double multitagTranslationStdDevCoefficient = 0.1;
    private final double multitagRotationStdDevCoefficient = 0.5;

    private AprilTagIO left;
    private PositionTargetIO limelight;

    private Optional<AprilTagIOInputs> leftInputs = Optional.empty();
    private Optional<PositionTargetIOInputs> limelightInputs = Optional.empty();

    public List<PhotonTrackedTarget> leftTargets = new ArrayList<>();

    public double leftInputsTimestamp = 0;

    private SwerveDriveSubsystem consumer;

    public boolean updatingPoseUsingVision = true;

    public VisionSubsystem(SwerveDriveSubsystem consumer, AprilTagIO left, PositionTargetIO limelight) {
        this.left = left;
        this.limelight = limelight;

        this.consumer = consumer;
    }

    public void periodic() {
        var tempInputs = left.updateInputs();
        leftInputs = tempInputs.getFirst();

        if (leftInputs.isPresent()) {
            leftInputsTimestamp = leftInputs.get().timestamp;
        }
        limelightInputs = limelight.updateInputs();

        leftTargets = tempInputs.getSecond();

        if (updatingPoseUsingVision) {
            addVisionPoseEstimate(leftInputs);
        }

        logVisionPoseEstimateInfo(leftInputs);

        Logger.log("/VisionSubsystem/isUsingVision", updatingPoseUsingVision);
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
        return limelightInputs.map(
                (inputs) -> new LimelightRawAngles(inputs.yaw, inputs.pitch, inputs.area, inputs.timestamp));
    }

    private void logVisionPoseEstimateInfo(Optional<AprilTagIOInputs> inputs) {
        inputs.ifPresent((t) -> {
            Logger.log("/VisionSubsystem/" + t.name + "/pose", t.poseEstimate3d, true);
            Logger.log("/VisionSubsystem/" + t.name + "/alternate", t.alternatePoseEstimate3d, true);
            Logger.log("/VisionSubsystem/" + t.name + "/targetNumber", t.numberOfTags);
            Logger.log("/VisionSubsystem/" + t.name + "/targetNumber", t.numberOfTags);
        });
    }

    private void addVisionPoseEstimate(Optional<AprilTagIOInputs> inputs) {

        inputs.ifPresent((t) -> {

            // distance from current pose to vision estimated pose
            double poseDifference = consumer.getPose()
                    .getTranslation()
                    .getDistance(t.poseEstimate3d.toPose2d().getTranslation());

            if (true) {
                double xyStds;
                double degStds;
                // multiple targets detected
                if (t.numberOfTags >= 2 && t.targetArea > 0.65) {
                    xyStds = 1;
                    degStds = 6;
                }

                if (t.numberOfTags >= 2 && t.targetArea > 0.075) {
                    xyStds = 2;
                    degStds = 12;
                }
                // 1 target with large area and close to estimated pose
                else if (t.targetArea > 0.8 && poseDifference < 0.5) {
                    xyStds = 2;
                    degStds = 12;
                }
                // 1 target farther away and estimated pose is close
                else if (t.targetArea > 0.1 && poseDifference < 0.3) {
                    xyStds = 4;
                    degStds = 30;
                }
                // conditions don't match to add a vision measurement
                else {
                    return;
                }

                consumer.addVisionMeasurement(
                        t.poseEstimate3d.toPose2d(),
                        t.timestamp,
                        MatBuilder.fill(Nat.N3(), Nat.N1(), xyStds, xyStds, Units.degreesToRadians(degStds)));
            }
        });
    }

    private void addVisionPoseEstimate(Pose3d estimate, double distance, double timestamp) {
        if (!isWithinField(estimate)) return;

        // TODO: acutlally calculate

        consumer.addVisionMeasurement(estimate.toPose2d(), timestamp, calculateVisionStdDevs(distance));
    }

    private void addVisionPoseEstimateTwoTags(Pose3d estimate, double distance, double timestamp) {
        if (!isWithinField(estimate)) return;

        // TODO: acutlally calculate

        consumer.addVisionMeasurement(estimate.toPose2d(), timestamp, calculateMultitagVisionStdDevs(distance));
    }

    private void addVisionPoseEstimateWithBackup(Pose3d estimate, Pose3d backup, double distance, double timestamp) {
        if (!isWithinField(estimate)) {
            addVisionPoseEstimate(backup, distance, timestamp);
        } else {
            addVisionPoseEstimate(estimate, distance, timestamp);
        }
    }

    private void addVisionPoseEstimateWithBackupMultitag(
            Pose3d estimate, Pose3d backup, double distance, double timestamp) {
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

        return isWithinField; // && isNearRobot;
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

    public Rotation2d getSpeakerAngleFromPose(Pose2d currentPose) {
        return FieldConstants.getSpeakerPose()
                .getTranslation()
                .minus(currentPose.getTranslation())
                .getAngle();
    }

    public Optional<Rotation2d> getSpeakerAngleFromVision(Pose2d currentPose) {
        return getSpeakerAngleFromVision(currentPose, true);
    }

    public Optional<Rotation2d> getSpeakerAngleFromVision(Pose2d currentPose, boolean timestampAdjust) {
        if (timestampAdjust) {
            currentPose = consumer.getPoseAtTimestamp(leftInputsTimestamp);
        }

        Optional<PhotonTrackedTarget> speakerTag =
                VisionSubsystem.getTagInfo(leftTargets, FieldConstants.getSpeakerTag());
        if (speakerTag.isPresent()) {
            return Optional.of(currentPose
                    .getRotation()
                    .plus(PinholeModel3D.getTranslationToTarget(
                                    new Translation3d(
                                            1,
                                            Math.tan(Math.toRadians(
                                                    -speakerTag.get().getYaw())),
                                            Math.tan(Math.toRadians(
                                                    speakerTag.get().getPitch()))),
                                    Constants.VisionConstants.robotToApriltagCamera,
                                    Units.inchesToMeters(57.75))
                            .getAngle()));
            // System.out.println(speakerTag.get().getYaw());
            // return Optional.of(currentPose
            //         .getRotation() // this is 182 not 180 because the camera is off by 2 ish degrees
            //         .plus(Rotation2d.fromDegrees(-speakerTag.get().getYaw()).plus(Rotation2d.fromDegrees(182))));
        } else {
            return Optional.empty();
        }
    }

    public double getSpeakerDistanceFromPose(Pose2d currentPose) {
        return FieldConstants.getSpeakerPose()
                .getTranslation()
                .minus(currentPose.getTranslation())
                .getNorm();
    }

    public OptionalDouble getSpeakerDistanceFromVision(Pose2d currentPose) {
        Optional<PhotonTrackedTarget> speakerTag =
                VisionSubsystem.getTagInfo(leftTargets, FieldConstants.getSpeakerTag());
        if (speakerTag.isPresent()) {
            return OptionalDouble.of(PinholeModel3D.getTranslationToTarget(
                            new Translation3d(
                                    1,
                                    Math.tan(Math.toRadians(-speakerTag.get().getYaw())),
                                    Math.tan(Math.toRadians(speakerTag.get().getPitch()))),
                            Constants.VisionConstants.robotToApriltagCamera,
                            Units.inchesToMeters(57.75))
                    .getNorm());
            // return OptionalDouble.of(PhotonUtils.calculateDistanceToTargetMeters(
            //         .56,
            //         Units.inchesToMeters(57.75),
            //         Units.degreesToRadians(34),
            //         Units.degreesToRadians(speakerTag.get().getPitch())));
        } else {
            return OptionalDouble.empty();
        }
    }
}
