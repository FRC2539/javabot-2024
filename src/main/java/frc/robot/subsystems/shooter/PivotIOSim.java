package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim extends PivotIOFalconTest {
    private final DCMotorSim motorSimModel = new DCMotorSim(DCMotor.getFalcon500(1), 1.0, 0.001);

    public PivotIOSim(int port, int encoderPort) {
        super(port, encoderPort);
    }

    public void update() {
        super.update();
        var talonFXSim = motor.getSimState();
        var motorVoltage = talonFXSim.getMotorVoltage();

        motorSimModel.setInputVoltage(motorVoltage);
        motorSimModel.update(0.020);

        talonFXSim.setRawRotorPosition(motorSimModel.getAngularPositionRotations());
        talonFXSim.setRotorVelocity(Units.radiansToRotations(motorSimModel.getAngularVelocityRadPerSec()));
    }

    protected Rotation2d getGripperEncoderAngle() {
        return getPositionAsRotation2d();
    }
}
