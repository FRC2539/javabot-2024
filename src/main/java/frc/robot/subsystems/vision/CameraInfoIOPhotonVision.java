package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;

public class CameraInfoIOPhotonVision implements CameraInfoIO {
    PhotonCamera camera;

    public CameraInfoIOPhotonVision(PhotonCamera photonCamera) {
        camera = photonCamera;
    }

    public Optional<CameraInfoIOInputs> updateInputs() {
        return Optional.empty();
    }

}
