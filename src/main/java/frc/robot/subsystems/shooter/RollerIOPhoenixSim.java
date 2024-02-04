package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class RollerIOPhoenixSim implements RollerIO {
    private TalonFXSimState talonFXSim;
    private TalonFX talonFX;

    private FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), ShooterConstants.gearRatioRoller, ShooterConstants.momentOfInertiaRoller);

    public RollerIOPhoenixSim(int port) {
        talonFX = new TalonFX(port, "CANivore");
        talonFXSim = talonFX.getSimState();
    }

    public void updateInputs(RollerIOInputs inputs) {
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        flywheelSim.setInputVoltage(talonFXSim.getMotorVoltage());
        flywheelSim.update(0.02);

        talonFXSim.setRotorVelocity(flywheelSim.getAngularVelocityRadPerSec()/(2 * Math.PI));

        inputs.speed = talonFX.getVelocity().getValue() * 60 * ShooterConstants.gearRatioRoller; //converts rps to rpm
        inputs.voltage = talonFX.getMotorVoltage().getValue();
        inputs.current = talonFX.getStatorCurrent().getValue();
        inputs.motorTemperature = talonFX.getDeviceTemp().getValue();
    }

    public void setSpeed(double speed) {
        talonFX.setControl(new VelocityVoltage(speed / 60 / ShooterConstants.gearRatioRoller));
    }
}