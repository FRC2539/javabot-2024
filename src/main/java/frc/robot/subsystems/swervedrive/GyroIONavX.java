package frc.robot.subsystems.swervedrive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class GyroIONavX implements GyroIO {
    private AHRS navX = new AHRS();

    public GyroIONavX() {}

    public void updateInputs(GyroIOInputs inputs) {
        inputs.rotation2d = navX.getRotation2d();
        inputs.rotation3d =
                new Rotation3d(Units.degreesToRadians(navX.getRoll()), Units.degreesToRadians(navX.getPitch()), 0);
        inputs.rotationRates3d = new Rotation3d(navX.getRawGyroX(), navX.getRawGyroY(), navX.getRawGyroZ());
        inputs.isActive = true;
    }
}
