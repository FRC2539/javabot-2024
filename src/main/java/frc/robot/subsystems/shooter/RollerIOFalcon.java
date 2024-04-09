package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

public class RollerIOFalcon implements RollerIO {
    private TalonFX talonFX;
    private final double gearRatio = 1;

    public RollerIOFalcon(int port) {
        talonFX = new TalonFX(port, "CANivore");
        talonFX.setInverted(false);

        Slot0Configs slot0Configs = new Slot0Configs();

        DCMotor exampleMotor = DCMotor.getFalcon500(1).withReduction(gearRatio);

        slot0Configs.kS = 0;
        // converts rads/s / V to V/rps
        slot0Configs.kV = 0.119; // 1 / (exampleMotor.KvRadPerSecPerVolt / Math.PI * 2);

        slot0Configs.kP = 0.2;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        talonFX.getConfigurator().apply(slot0Configs);

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        talonFX.getConfigurator().refresh(currentLimitsConfigs);
        currentLimitsConfigs.StatorCurrentLimit = 80;
        currentLimitsConfigs.SupplyCurrentLimit = 90;
        talonFX.getConfigurator().apply(currentLimitsConfigs);

        talonFX.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(RollerIOInputs inputs) {
        inputs.speed = talonFX.getVelocity().getValue() * 2 * Math.PI; // converts rps to rpm
        inputs.voltage = talonFX.getMotorVoltage().getValue();
        inputs.current = talonFX.getStatorCurrent().getValue();
        inputs.motorTemperature = talonFX.getDeviceTemp().getValue();
    }

    public void setSpeed(double speed) {
        talonFX.setControl(new VelocityVoltage(speed / (Math.PI * 2)));
    }

    public void setVoltage(double voltage) {
        talonFX.setControl(new VoltageOut(voltage));
    }
}
