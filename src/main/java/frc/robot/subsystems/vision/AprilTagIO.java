package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagIO extends CameraIO<AprilTagIO.AprilTagIOInputs> {
    public Optional<AprilTagIOInputs> updateInputs();
    
    public static class AprilTagIOInputs {
        Pose3d poseEstimate3d;
        double targetDistance;
        double timestamp;
    }
}
