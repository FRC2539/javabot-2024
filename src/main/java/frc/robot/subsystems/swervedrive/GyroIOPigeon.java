package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon implements GyroIO {
    private Pigeon2 pigeon;

    public GyroIOPigeon(int port, String canbus) {
        pigeon = new Pigeon2(port, canbus);
    }

    public GyroIOPigeon(int port) {
        pigeon = new Pigeon2(port);
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.rotation2d = Rotation2d.fromDegrees(pigeon.getYaw());

        // offsets the values by a bit because the Pigeon is not perfectly mounted
        inputs.rotation3d = new Rotation3d(
                Units.degreesToRadians(pigeon.getRoll()) + 0.019, Units.degreesToRadians(pigeon.getPitch()) - 0.082, 0);

        double[] rawXYZ = new double[3];

        pigeon.getRawGyro(rawXYZ);
        inputs.rotationRates3d = new Rotation3d(rawXYZ[0], rawXYZ[1], rawXYZ[2]);

        inputs.isActive = true;
    }
}
