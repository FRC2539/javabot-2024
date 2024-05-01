package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.lib.framework.motor.MotorIOTalonFX;

public class RollerIOFalcon extends MotorIOTalonFX {

    public RollerIOFalcon(int port) {
        super(port, "CANivore");
        motor.setInverted(false);

        Slot0Configs slot0Configs = new Slot0Configs();

        slot0Configs.kS = 0;
        // converts rads/s / V to V/rps
        slot0Configs.kV = 0.119;

        slot0Configs.kP = 0.2;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        motor.getConfigurator().apply(slot0Configs);
    }
}
