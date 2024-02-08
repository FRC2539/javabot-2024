package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;

public class RollerIOSim implements RollerIO{
    private double RPM;

    public RollerIOSim() {};

    public void updateInputs(RollerIOInputs inputs) {
        inputs.speed = RPM;
    }

    public void setSpeed(double RPM) {
        this.RPM = RPM;
    }

    public void setVoltage(double voltage) {
        this.RPM = DCMotor.getFalcon500(1).getSpeed(0,voltage) / (2 * Math.PI) * 60;
    }

}
