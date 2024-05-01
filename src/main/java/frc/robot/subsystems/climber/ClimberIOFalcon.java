package frc.robot.subsystems.climber;

import frc.lib.framework.motor.MotorIOTalonFX;

public class ClimberIOFalcon extends MotorIOTalonFX {
    public ClimberIOFalcon() {
        super(16, "CANivore");
        motor.setInverted(true);
        zeroPosition(0);
    }
}
