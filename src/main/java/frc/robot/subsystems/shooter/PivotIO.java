package frc.robot.subsystems.shooter;

import frc.lib.framework.motor.MotorIO;

public interface PivotIO extends MotorIO {
    public boolean isEncoderConnected();
}
