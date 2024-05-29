package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOFalcon implements IntakeIO {
    private double chamberSpeed;
    private double rollerSpeed;

    private boolean hasSeenChamber = false;
    private boolean hasSeenRoller = false;

    private final boolean isThreading = false;

    private final double percentMaxOutput = 1;

    private TalonFX chamberMotor = new TalonFX(IntakeConstants.chamberMotorPort, "CANivore");
    private TalonFX rollerMotor = new TalonFX(IntakeConstants.rollerMotorPort, "CANivore");

    private AnalogInput rollerSensor = new AnalogInput(IntakeConstants.rollerSensorPort);
    private AnalogInput chamberSensor = new AnalogInput(IntakeConstants.chamberSensorPort);

    private LinearFilter average = LinearFilter.movingAverage(10);

    public IntakeIOFalcon() {
        if (isThreading) {
            Thread updatingThread = new Thread(new SubRunner());

            updatingThread.start();
        }

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        chamberMotor.getConfigurator().refresh(currentLimitsConfigs);
        currentLimitsConfigs.StatorCurrentLimit = 80;
        currentLimitsConfigs.SupplyCurrentLimit = 90;
        chamberMotor.getConfigurator().apply(currentLimitsConfigs);

        CurrentLimitsConfigs currentLimitsConfigsOther = new CurrentLimitsConfigs();
        rollerMotor.getConfigurator().refresh(currentLimitsConfigsOther);
        currentLimitsConfigsOther.StatorCurrentLimit = 80;
        currentLimitsConfigsOther.SupplyCurrentLimit = 90;
        rollerMotor.getConfigurator().apply(currentLimitsConfigsOther);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.chamberSpeed = chamberSpeed;
        inputs.rollerSpeed = rollerSpeed;

        if (isThreading) {
            inputs.chamberSensor = hasSeenChamber;
            inputs.rollerSensor = hasSeenRoller;

            hasSeenChamber = false;
            hasSeenRoller = false;
        } else {
            inputs.chamberSensor = hasChamberPiece();
            inputs.rollerSensor = hasRollerPiece();
        }

        inputs.chamberCurrent = chamberMotor.getTorqueCurrent().getValueAsDouble();
        inputs.rollerCurrent = rollerMotor.getTorqueCurrent().getValueAsDouble();
    }

    private boolean hasRollerPiece() {
        return rollerSensor.getValue() < 50;
    }

    private boolean hasChamberPiece() {
        return chamberSensor.getValue() < 50;
    }

    public void setRollerSpeed(double speed) {
        rollerSpeed = speed;
        rollerMotor.setVoltage(percentMaxOutput * speed * 12);
    }

    public void setChamberSpeed(double speed) {
        chamberSpeed = speed;
        chamberMotor.setVoltage(percentMaxOutput * speed * 12);
    }

    private class SubRunner implements Runnable {
        public void run() {
            while (true) {
                try {
                    Thread.sleep(2);
                    if (hasRollerPiece()) {
                        hasSeenRoller = true;
                    }
                    if (hasChamberPiece()) {
                        hasSeenChamber = true;
                    }
                } catch (Exception e) {

                }
            }
        }
    }
}
