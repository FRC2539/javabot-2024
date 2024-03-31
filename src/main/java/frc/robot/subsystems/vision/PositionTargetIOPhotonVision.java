package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PositionTargetIOPhotonVision implements PositionTargetIO {

    PhotonCamera camera;

    boolean invertPitchAndYaw;

    public PositionTargetIOPhotonVision(PhotonCamera camera, boolean invertPitchAndYaw) {
        this.camera = camera;
        this.invertPitchAndYaw = invertPitchAndYaw;
    }

    public Optional<PositionTargetIOInputs> updateInputs() {
        try {
            PhotonPipelineResult result = camera.getLatestResult();
            if (!result.hasTargets()) {
                return Optional.empty();
            }

            var myThingy = new PositionTargetIOInputs();
            myThingy.pitch = result.getBestTarget().getPitch();
            myThingy.yaw = result.getBestTarget().getYaw() * (invertPitchAndYaw ? -1 : 1);
            myThingy.timestamp = result.getTimestampSeconds();

            return Optional.of(myThingy);
        } catch (Exception e) {
            return Optional.empty();
        }
    }

    public String getName() {
        return "limelight";
    }
}
