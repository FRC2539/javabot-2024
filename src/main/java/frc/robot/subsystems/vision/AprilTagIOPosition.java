package frc.robot.subsystems.vision;

import java.util.Optional;

public class AprilTagIOPosition implements AprilTagIO {
    private CameraInfoIO cameraInfoIO;

    public AprilTagIOPosition(CameraInfoIO cameraInfo) {
        this.cameraInfoIO = cameraInfo;
    }

    public Optional<AprilTagIOInputs> updateInputs() {
        return cameraInfoIO.updateInputs().map(
            inputs -> {
                AprilTagIOInputs myInputs = new AprilTagIOInputs();
                myInputs.poseEstimate = inputs.poseEstimate3d.toPose2d();
                myInputs.poseEstimate3d = inputs.poseEstimate3d;
                myInputs.targetDistance = inputs.targetDistance;
                myInputs.timestamp = inputs.timestamp;

                return myInputs;
            }
        );
    }
}
