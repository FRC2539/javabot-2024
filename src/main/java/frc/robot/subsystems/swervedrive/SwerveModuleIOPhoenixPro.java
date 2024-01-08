package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.Conversions;
import frc.lib.swerve.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModuleIOPhoenixPro implements SwerveModuleIO {
    public int moduleNumber;
    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANcoder angleEncoder;

    private VoltageOut voltageRequestDrive = new VoltageOut(0).withEnableFOC(true);

    private PositionVoltage positionVoltageRequestAngle =
            new PositionVoltage(0).withSlot(0).withEnableFOC(true);
    private VelocityVoltage velocityVoltageRequestDrive =
            new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

    SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
            Constants.SwerveConstants.calculatedDriveKS,
            Constants.SwerveConstants.calculatedDriveKV,
            Constants.SwerveConstants.calculatedDriveKA);

    StatusSignal<Double> drivePositionSS;
    StatusSignal<Double> driveVelocitySS;
    StatusSignal<Double> driveRotorVelocitySS;
    StatusSignal<Double> driveTemperatureSS;
    StatusSignal<Double> driveVoltageSS;
    StatusSignal<Double> driveCurrentSS;

    StatusSignal<Double> anglePositionSS;
    StatusSignal<Double> angleVelocitySS;
    StatusSignal<Double> angleTemperatureSS;
    StatusSignal<Double> angleVoltageSS;
    StatusSignal<Double> angleCurrentSS;

    StatusSignal<Double> encoderAbsoluteAngleSS;

    // Testing a calculation method
    // SimpleMotorFeedforward angleFeedforward = new SimpleMotorFeedforward(
    //         Constants.SwerveConstants.angleKS, Constants.SwerveConstants.angleKV, Constants.SwerveConstants.angleKA);

    public SwerveModuleIOPhoenixPro(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;


        /* Angle Encoder Config */
        angleEncoder = moduleConstants.canivoreName.isEmpty()
                ? new CANcoder(moduleConstants.cancoderID)
                : new CANcoder(moduleConstants.cancoderID, moduleConstants.canivoreName.get());
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = moduleConstants.canivoreName.isEmpty()
                ? new TalonFX(moduleConstants.angleMotorID)
                : new TalonFX(moduleConstants.angleMotorID, moduleConstants.canivoreName.get());
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = moduleConstants.canivoreName.isEmpty()
                ? new TalonFX(moduleConstants.driveMotorID)
                : new TalonFX(moduleConstants.driveMotorID, moduleConstants.canivoreName.get());
        configDriveMotor();

        drivePositionSS = driveMotor.getPosition();
        driveVelocitySS = driveMotor.getVelocity();
        driveRotorVelocitySS = driveMotor.getRotorVelocity();
        driveTemperatureSS = driveMotor.getDeviceTemp();
        driveVoltageSS = driveMotor.getSupplyVoltage();
        driveCurrentSS = driveMotor.getSupplyCurrent();

        anglePositionSS = angleMotor.getPosition();
        angleVelocitySS = angleMotor.getVelocity();
        angleTemperatureSS = angleMotor.getDeviceTemp();
        angleVoltageSS = angleMotor.getSupplyVoltage();
        angleCurrentSS = angleMotor.getSupplyCurrent();

        encoderAbsoluteAngleSS = angleEncoder.getAbsolutePosition();
    }

    public void setDesiredVelocityOpenLoop(double velocity) {
        double percentOutput = velocity / Constants.SwerveConstants.maxSpeed;
        driveMotor.setControl(voltageRequestDrive.withOutput(percentOutput * Constants.GlobalConstants.targetVoltage));
    }

    public void setDesiredVelocityClosedLoop(double velocity) {
        double velocityMotor = Conversions.MPSToMotorRPS(
                velocity, Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
        driveMotor.setControl(velocityVoltageRequestDrive
                .withVelocity(velocityMotor)
                .withFeedForward(driveFeedforward.calculate(velocity)));
    }

    public void setDesiredAngularPosition(double angularPosition) {
        angleMotor.setControl(
                positionVoltageRequestAngle.withPosition(angularPosition / 360).withFeedForward(0));
    }

    public void setDesiredAngularPositionAndVelocity(double angularPosition, double angularVelocity) {
        angleMotor.setControl(positionVoltageRequestAngle
                .withPosition(angularPosition / 360)
                .withFeedForward(angularVelocity * Constants.SwerveConstants.calculatedAngleKV));
    }

    public void disableDriveMotor() {
        driveMotor.stopMotor();
    }

    public void disableAngleMotor() {
        angleMotor.stopMotor();
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.angularVelocity = Conversions.motorRPSToRadPS(
                angleVelocitySS.refresh().getValue(), Constants.SwerveConstants.angleGearRatio);
        inputs.velocity = Conversions.motorRPSToMPS(
                driveRotorVelocitySS.refresh().getValue(),
                Constants.SwerveConstants.wheelCircumference,
                Constants.SwerveConstants.driveGearRatio);
        inputs.angularPosition = Rotation2d.fromRotations(
                BaseStatusSignal.getLatencyCompensatedValue(anglePositionSS.refresh(), angleVelocitySS.refresh()));
        inputs.position = Conversions.motorToMeters(
                BaseStatusSignal.getLatencyCompensatedValue(drivePositionSS.refresh(), driveVelocitySS.refresh()),
                Constants.SwerveConstants.wheelCircumference,
                Constants.SwerveConstants.driveGearRatio);
        inputs.encoderAngle =
                Rotation2d.fromRotations(encoderAbsoluteAngleSS.refresh().getValue());

        inputs.driveTemperature = driveTemperatureSS.refresh().getValue();
        inputs.angleTemperature = angleTemperatureSS.refresh().getValue();
        inputs.driveVoltage = driveVoltageSS.refresh().getValue();
        inputs.angleVoltage = angleVoltageSS.refresh().getValue();
        inputs.driveCurrent = driveCurrentSS.refresh().getValue();
        inputs.angleCurrent = angleCurrentSS.refresh().getValue();
    }

    private void configAngleEncoder() {
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
        MagnetSensorConfigs myMagnetSensorConfigs = new MagnetSensorConfigs();
        angleEncoder.getConfigurator().refresh(myMagnetSensorConfigs);
        angleEncoder.getConfigurator().apply(myMagnetSensorConfigs);
    }

    private void configAngleMotor() {
        TalonFXConfigurator configurator = angleMotor.getConfigurator();
        configurator.apply(Robot.ctreConfigs.swerveAngleFXConfig);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        outputConfigs.Inverted = Constants.SwerveConstants.angleMotorInvert
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        outputConfigs.NeutralMode = Constants.SwerveConstants.angleNeutralMode;

        configurator.apply(outputConfigs);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedbackConfigs.RotorToSensorRatio = Constants.SwerveConstants.angleGearRatio;
        configurator.apply(feedbackConfigs);
    }

    private void configDriveMotor() {
        TalonFXConfigurator configurator = driveMotor.getConfigurator();
        configurator.apply(Robot.ctreConfigs.swerveDriveFXConfig);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        outputConfigs.NeutralMode = Constants.SwerveConstants.driveNeutralMode;
        outputConfigs.Inverted = Constants.SwerveConstants.driveMotorInvert
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        configurator.apply(outputConfigs);
    }
}
