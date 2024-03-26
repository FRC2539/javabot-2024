package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.lib.vision.LimelightRawAngles;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.AprilTagIO.AprilTagIOInputs;
import frc.robot.subsystems.vision.PositionTargetIO.PositionTargetIOInputs;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonUtils;
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

    private SwerveDriveSubsystem consumer;

    public boolean usingVision = true;

    public VisionSubsystem(SwerveDriveSubsystem consumer, AprilTagIO left, PositionTargetIO limelight) {
        this.left = left;
        this.limelight = limelight;

        this.consumer = consumer;
    }

    public void periodic() {
        leftInputs = left.updateInputs();
        limelightInputs = limelight.updateInputs();

        leftTargets = leftInputs.orElse(new AprilTagIOInputs()).targets;

        if (usingVision) {
            addVisionPoseEstimate(leftInputs);
        }

        logVisionPoseEstimateInfo(leftInputs);

        Logger.log("/VisionSubsystem/isUsingVision", usingVision);
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
        return limelightInputs.map((inputs) -> new LimelightRawAngles(inputs.yaw, inputs.pitch, inputs.area, inputs.timestamp - Timer.getFPGATimestamp()));
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
                if (t.numberOfTags >= 2 && t.targetArea > 0.8) {
                    xyStds = 1;
                    degStds = 6;
                }

                if (t.numberOfTags >= 2 && t.targetArea > 0.1) {
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

    public boolean hasTag = false;
    private static boolean usingCameraDirectly = true;
    public Rotation2d lastSpeakerAngle = new Rotation2d(Math.PI);

    public Rotation2d getSpeakerAngle(Pose2d currentPose) {
        return getSpeakerAngle(currentPose, false);
    }

    public Rotation2d getSpeakerAngle(Pose2d currentPose, boolean useVision) {
        
            lastSpeakerAngle = FieldConstants.getSpeakerPose()
                    .getTranslation()
                    .minus(currentPose.getTranslation())
                    .getAngle();
                    if (!useVision) {
            return lastSpeakerAngle;}

        Optional<PhotonTrackedTarget> speakerTag =
                VisionSubsystem.getTagInfo(leftTargets, FieldConstants.getSpeakerTag());
        // lastSpeakerAngle =
        // FieldConstants.getSpeakerPose().getTranslation().minus(currentPose.getTranslation()).getAngle();
        Logger.log("/VisionSubsystem/LastSpeakerAngle", lastSpeakerAngle.getRadians());
        if (speakerTag.isPresent() && usingCameraDirectly) {
            hasTag = true;

            // TODO: This is not right but at least it mihgt works
            // double distance = getSpeakerDistance(currentPose);
            // Transform2d transformRobotToCamera = new Transform2d(
            //     VisionConstants.robotToLeftCamera.getTranslation().toTranslation2d(),
            //     Rotation2d.fromDegrees(19));
            // Transform2d transformCameraToGoal =
            //         new Transform2d(0,0,Rotation2d.fromDegrees(-speakerTag.get().getYaw()))
            //         .plus(new Transform2d(distance, 0, new Rotation2d()));

            // Transform2d transformRobotToGoal = transformRobotToCamera.plus(transformCameraToGoal);

            // Gets that final translation to goal and
            // lastSpeakerAngle = currentPose.getRotation().plus(transformRobotToGoal.getTranslation().getAngle());
            lastSpeakerAngle = currentPose
                    .getRotation()
                    .plus(Rotation2d.fromDegrees(-speakerTag.get().getYaw()).plus(Rotation2d.fromDegrees(180)));
            Logger.log("/VisionSubsystem/LastSpeakerAngleDirect", lastSpeakerAngle.getRadians());
        } else {
            Optional<PhotonTrackedTarget> altSpeakerTag =
                    VisionSubsystem.getTagInfo(leftTargets, FieldConstants.getSpeakerTag());
            if (altSpeakerTag.isPresent() && usingCameraDirectly) {

                // TODO: This is not right but at least it mihgt works
                // double distance = getSpeakerDistance(currentPose);
                // Transform2d transformRobotToCamera = new Transform2d(
                //     VisionConstants.robotToLeftCamera.getTranslation().toTranslation2d(),
                //     Rotation2d.fromDegrees(19));
                // Transform2d transformCameraToGoal =
                //         new Transform2d(0,0,Rotation2d.fromDegrees(-speakerTag.get().getYaw()))
                //         .plus(new Transform2d(distance, 0, new Rotation2d()));

                // Transform2d transformRobotToGoal = transformRobotToCamera.plus(transformCameraToGoal);

                // Gets that final translation to goal and
                // lastSpeakerAngle = currentPose.getRotation().plus(transformRobotToGoal.getTranslation().getAngle());
                lastSpeakerAngle = currentPose
                        .getRotation()
                        .plus(Rotation2d.fromDegrees(-altSpeakerTag.get().getYaw())
                                .plus(Rotation2d.fromDegrees(180)));
                Logger.log("/VisionSubsystem/LastSpeakerAngleDirect", lastSpeakerAngle.getRadians());
            }
            hasTag = false;
        }
        return lastSpeakerAngle;
    }

    public double lastDistance = 3;

    public double getSpeakerDistance(Pose2d currentPose) {
        return getSpeakerDistance(currentPose, false);
    }

    public double getSpeakerDistance(Pose2d currentPose, boolean useVision) {
        
            lastDistance = FieldConstants.getSpeakerPose()
                    .getTranslation()
                    .minus(currentPose.getTranslation())
                    .getNorm();
            Logger.log("/VisionSubsystem/LastSpeakerDistance", lastDistance);
        
        if (!useVision) {
            return lastDistance;
        }

        Optional<PhotonTrackedTarget> speakerTag =
                VisionSubsystem.getTagInfo(leftTargets, FieldConstants.getSpeakerTag());
        // lastDistance =
        // FieldConstants.getSpeakerPose().getTranslation().minus(currentPose.getTranslation()).getNorm();
        Logger.log("/VisionSubsystem/LastSpeakerDistance", lastDistance);
        if (speakerTag.isPresent() && usingCameraDirectly) {
            // TODO: This is not right
            lastDistance = PhotonUtils.calculateDistanceToTargetMeters(
                    .56, Units.inchesToMeters(57.75),
                    Units.degreesToRadians(34),
                            Units.degreesToRadians(speakerTag.get().getPitch()));
            Logger.log("/VisionSubsystem/LastSpeakerDistanceDirect", lastDistance);
        }
        return lastDistance;
    }
}
