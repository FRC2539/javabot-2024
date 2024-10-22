package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.lib.vision.LimelightRawAngles;
import frc.lib.vision.PinholeModel3D;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    public static double translationStdDevCoefficient = 1;
    public static double rotationStdDevCoefficient = 1.2;

    private static final double multitagTranslationStdDevCoefficient = 0.1;
    private static final double multitagRotationStdDevCoefficient = 0.5;

    private SwerveDriveSubsystem consumer;

    public boolean updatingPoseUsingVision = true;

    private final String aprilTagCameraName = "limelight-april";
    private final String positionTargetCameraName = "limelight-intake";

    private PoseEstimate visionPoseEstimate = new PoseEstimate();

    private Optional<LimelightRawAngles> noteLimelightResults = Optional.empty();

    public VisionSubsystem(SwerveDriveSubsystem consumer) {

        this.consumer = consumer;
    }

    public void periodic() {

        visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(aprilTagCameraName);

        if (LimelightHelpers.getTV(positionTargetCameraName)) {
            noteLimelightResults = Optional.of(
                new LimelightRawAngles(
                    LimelightHelpers.getTX(positionTargetCameraName),
                    LimelightHelpers.getTY(positionTargetCameraName),
                    LimelightHelpers.getTA(positionTargetCameraName),
                    LimelightHelpers.getLatency_Pipeline(positionTargetCameraName) / 1000.0 + Timer.getFPGATimestamp())
            );
        } else {
            noteLimelightResults = Optional.empty();
        }

        updatePoseUsingVision();

        logVisionPoseEstimateInfo();

        Logger.log("/VisionSubsystem/isUsingVision", updatingPoseUsingVision);
    }

    private void updatePoseUsingVision() {

        var results = visionPoseEstimate;

    
        @SuppressWarnings("unused")
        double poseDifference = consumer.getPose()
                .getTranslation()
                .getDistance(results.pose.getTranslation());

        double xyStds;
        double degStds;
        if (consumer.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond > Math.PI * 4) {
            return;
        }

        if (results.tagCount == 0) {
            return;
        }

        xyStds = 0.3;
        degStds = Units.radiansToDegrees(9999999);

        consumer.addVisionMeasurement(
                results.pose,
                results.timestampSeconds,
                MatBuilder.fill(Nat.N3(), Nat.N1(), xyStds, xyStds, Units.degreesToRadians(degStds)));
    }

    @SuppressWarnings("unused")
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

    public static Matrix<N3, N1> calculateVisionStdDevs(double distance) {
        var translationStdDev = translationStdDevCoefficient * distance;
        var rotationStdDev = rotationStdDevCoefficient * distance;

        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }

    public static Matrix<N3, N1> calculateMultitagVisionStdDevs(double distance) {
        var translationStdDev = multitagTranslationStdDevCoefficient * distance;
        var rotationStdDev = multitagRotationStdDevCoefficient * distance;

        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }

    public Optional<LimelightRawAngles> getDetectorInfo() {
        
        return noteLimelightResults;
    }

    public Optional<RawFiducial> getTagInfo(int tagID) {
        for (var target : visionPoseEstimate.rawFiducials) {
            if (target.id == tagID) {
                return Optional.of(target);
            }
        }
        return Optional.empty();
    }

    public Optional<Pose2d> getPoseFromLimelight(boolean compensateForLatency) {
        boolean hasTags = visionPoseEstimate.tagCount > 0;

        if (hasTags) {
            if (compensateForLatency) {
                return Optional.of(compensateForLatency(visionPoseEstimate.pose, visionPoseEstimate.timestampSeconds));
            } else {
                return Optional.of(visionPoseEstimate.pose);
            }
        }
        
        return Optional.empty();
    }

    public Optional<Pose2d> getPoseFromPinhole(boolean compensateForLatency) {
        Optional<RawFiducial> speakerTag =
            getTagInfo(FieldConstants.getSpeakerTag());

        return speakerTag.map((target) -> {
            Translation2d robotToTarget = PinholeModel3D.getTranslationToTarget(
                                new Translation3d(
                                        1,
                                        Math.tan(Math.toRadians(
                                                -target.txnc)),
                                        Math.tan(Math.toRadians(
                                                target.tync))),
                                VisionConstants.robotToApriltagCamera,
                                Units.inchesToMeters(57.75));
            Rotation2d originToRobotRotation = consumer.getPose().getRotation();

            Translation2d originToRobot = FieldConstants.getSpeakerPose().getTranslation().minus(robotToTarget.rotateBy(originToRobotRotation));

            Pose2d calulatedPose = new Pose2d(originToRobot, originToRobotRotation);

            if (compensateForLatency) {
                return compensateForLatency(calulatedPose, visionPoseEstimate.timestampSeconds);
            } else {
                return calulatedPose;
            }
        });
    }

    public Pose2d compensateForLatency(Pose2d measuredPose, double timestamp) {
        return measuredPose.plus(consumer.getPose().minus(consumer.getPoseAtTimestamp(timestamp)));
    }

    private void logVisionPoseEstimateInfo() {
        if (visionPoseEstimate.tagCount == 0) return;

        Logger.log("/VisionSubsystem/" + aprilTagCameraName + "/pose", visionPoseEstimate.pose);
        Logger.log("/VisionSubsystem/" + aprilTagCameraName + "/targetNumber", visionPoseEstimate.tagCount);
    }
}
