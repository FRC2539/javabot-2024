package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.MathUtils;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.AprilTagIO.AprilTagIOInputs;

public class VisionSubsystem extends SubsystemBase {
    private final double translationStdDevCoefficient = 0.3;
    private final double rotationStdDevCoefficient = 0.9;

    private AprilTagIO[] cameras;
    private Optional<AprilTagIOInputs>[] inputs;

    private SwerveDriveSubsystem consumer;

    @SuppressWarnings("unchecked")
    public VisionSubsystem(SwerveDriveSubsystem consumer, AprilTagIO... cameras) {
        this.cameras = cameras;
        inputs = (Optional<AprilTagIOInputs>[]) new Optional[cameras.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = Optional.empty();
        }
        this.consumer = consumer;
    }

    public void periodic() {
        for (int i = 0; i < cameras.length; i++) {
            inputs[i] = cameras[i].updateInputs();

            inputs[i].ifPresent(t -> addVisionPoseEstimate(t.poseEstimate3d, t.targetDistance, t.timestamp));
        }
    }

    private void addVisionPoseEstimate(Pose3d estimate, double distance, double timestamp) {
        if (!isValidPose(estimate)) return;

        consumer.addVisionPoseEstimate(
            estimate.toPose2d(), timestamp, calculateVisionStdDevs(distance));
    }

    private boolean isValidPose(Pose3d pose) {
        boolean isWithinField = MathUtils.isInRange(pose.getY(), -5, FieldConstants.fieldWidth + 5)
                && MathUtils.isInRange(pose.getX(), -5, FieldConstants.fieldLength + 5)
                && MathUtils.isInRange(pose.getZ(), 0, 5);

        boolean isNearRobot = consumer
                        .getPose()
                        .getTranslation()
                        .getDistance(pose.getTranslation().toTranslation2d())
                < 1.4;

        return isWithinField && isNearRobot;
    }

    private Matrix<N3, N1> calculateVisionStdDevs(double distance) {
        var translationStdDev = translationStdDevCoefficient * Math.pow(distance, 2);
        var rotationStdDev = rotationStdDevCoefficient * Math.pow(distance, 2);

        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }
}
