package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface GyroIO {

    public void updateInputs(GyroIOInputs inputs);

    public class GyroIOInputs {
        public Rotation2d rotation2d = new Rotation2d();
        /** This is the rotation with the yaw set to 0. */
        public Rotation3d rotation3d = new Rotation3d();

        public Rotation3d rotationRates3d = new Rotation3d();

        public boolean isActive = false;
    }
}
