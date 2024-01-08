package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface CameraInfoIO {
    public Optional<CameraInfoIOInputs> updateInputs();
    
    public static class CameraInfoIOInputs {
        Pose3d poseEstimate3d;
        Pose2d poseEstimate;
        double targetDistance;
        double timestamp;
        
        double tx; //tx is positive right
        double ty; //ty is positive up

        int targetID;
    }
}
