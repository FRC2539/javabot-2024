package frc.robot.subsystems.vision;

import java.util.Optional;

import frc.robot.subsystems.vision.AprilTagIO.AprilTagIOInputs;

public class AprilTagIOPosition {
    private CameraInfoIO cameraInfoIO;

    public AprilTagIOPosition(CameraInfoIO cameraInfo) {
        this.cameraInfoIO = cameraInfo;
    }

    public Optional<AprilTagIOInputs> updateInputs() {
        return cameraInfoIO.updateInputs().map(
            inputs -> {
                AprilTagIOInputs myInputs = new AprilTagIOInputs();
                myInputs.poseEstimate3d = inputs.poseEstimate3d;
                myInputs.targetDistance = inputs.targetDistance;
                myInputs.timestamp = inputs.timestamp;

                return myInputs;
            }
        );
    }

    public String getName() {
        return "test";
    }
}
