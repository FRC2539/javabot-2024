package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class RollerIOSim extends RollerIOFalcon {
    private final DCMotorSim motorSimModel = new DCMotorSim(DCMotor.getFalcon500(1), 1.0, 0.001);
    FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.001);

    public RollerIOSim(int port) {
        super(port);
    }

    public void update() {
        var talonFXSim = motor.getSimState();
        var motorVoltage = talonFXSim.getMotorVoltage();

        motorSimModel.setInputVoltage(motorVoltage);
        motorSimModel.update(0.020);

        talonFXSim.setRawRotorPosition(motorSimModel.getAngularPositionRotations());
        talonFXSim.setRotorVelocity(Units.radiansToRotations(motorSimModel.getAngularVelocityRadPerSec()));
    }
}
