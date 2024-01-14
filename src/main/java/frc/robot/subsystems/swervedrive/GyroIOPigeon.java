package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.hardware.Pigeon2;  
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
        inputs.rotation2d = Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());

        // offsets the values by a bit because the Pigeon is not perfectly mounted
        // TODO: ADD LATENCY COMPENSATION
        inputs.rotation3d = new Rotation3d(
                Units.degreesToRadians(pigeon.getRoll().getValueAsDouble()) + 0.019, Units.degreesToRadians(pigeon.getPitch().getValueAsDouble()) - 0.082, 0);

        inputs.rotationRates3d = new Rotation3d(pigeon.getAngularVelocityXDevice().getValueAsDouble(), pigeon.getAngularVelocityYDevice().getValueAsDouble(), pigeon.getAngularVelocityZDevice().getValueAsDouble());

        inputs.isActive = true;
    }
}
