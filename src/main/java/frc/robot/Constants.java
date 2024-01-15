package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.swerve.SecondOrderSwerveKinematics;
import frc.lib.swerve.SwerveModuleConstants;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public final class Constants {
    public static final boolean competitionMode = false;

    public static final class GlobalConstants {
        public static final String CANIVORE_NAME = "CANivore";
        public static final int PCM_ID = 17;
        public static final double targetVoltage = 12.0; // Used for voltage compensation
    }

    public static final class ControllerConstants {
        public static final int LEFT_DRIVE_CONTROLLER = 0;
        public static final int RIGHT_DRIVE_CONTROLLER = 1;
        public static final int OPERATOR_CONTROLLER = 2;
    }

    public static final class TimesliceConstants {
        public static final double ROBOT_PERIODIC_ALLOCATION = 0.004;
        public static final double CONTROLLER_PERIOD = 0.010;

        public static final double DRIVETRAIN_PERIOD = 0.0025;
    }

    public static final class ShooterConstants {
        public static final int rightShooterPort = 8;
        public static final int leftShooterPort = 9;
        public static final int rightPivotPort = 10;
        public static final int leftPivotPort = 11;

        public static final int encoderPort = 1;

        public static final int shooterSensorPort = 1;
    }

    public static final class SwerveConstants extends DevelopmentBotConstants {}

    public static class CompBotConstants {

        // See https://github.com/Team364/BaseFalconSwerve for getting these values.
        // copy and pasted from comp bot swerve constants because i am illiterate and didn't know what to add/what not
        // to add
        public static final boolean hasPigeon = true;
        public static final int PIGEON_PORT = 29;

        public static final double lengthWithBumpers = Units.inchesToMeters(26 + 3.25 * 2);
        public static final double widthWithBumpers = Units.inchesToMeters(26 + 3.25 * 2);

        public static final double trackWidth = Units.inchesToMeters(19.5);
        public static final double wheelBase = Units.inchesToMeters(19.5);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double robotMass = Units.lbsToKilograms(115);

        public static final double openLoopRamp = 0.0; // 0.25
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (5.14 / 1.0); // 5.14:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };

        public static final SecondOrderSwerveKinematics swerveKinematics =
                new SecondOrderSwerveKinematics(moduleTranslations);

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.2;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Motor Information */
        public static final double driveMotorFreeSpeed = 6380; // RPM of Falcon 500
        public static final double angleMotorFreeSpeed = 6380; // RPM of Falcon 500
        public static final double stallTorque = 4.69;

        /* Drive Motor Characterization Values */
        public static final double driveKS =
                (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Angle Motor Characterization Values */
        public static final double angleKS = 0;
        // (0.368 / 12); // divide by 12 to convert from volts to percent output for CTRE
        public static final double angleKV = (0.234 / 12);
        public static final double angleKA = (0.003 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 6.52; // meters per second
        public static final double maxAcceleration =
                (stallTorque * driveGearRatio * 4) / (wheelDiameter * robotMass); // 16.52; // meters per second^2
        public static final double maxAngularVelocity = maxSpeed // rad/s
                / Arrays.stream(moduleTranslations)
                        .map(translation -> translation.getNorm())
                        .max(Double::compare)
                        .get();

        /* Calculated Characterization Values */
        public static final double calculatedDriveKS = 0;
        public static final double calculatedDriveKV = (12 / maxSpeed) / GlobalConstants.targetVoltage;
        public static final double calculatedDriveKA = (12 / maxAcceleration) / GlobalConstants.targetVoltage;
        public static final double calculatedAngleKV =
                (12 * 60) / (angleMotorFreeSpeed * Math.toRadians(360 / angleGearRatio));

        /* Precise Driving Mode Values */
        public static final double preciseDrivingModeSpeedMultiplier = 0.2;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Drive Motor Inverts */
        public static final boolean driveMotorInvert = true;

        /* Drive Encoder Inverts */
        public static final boolean driveEncoderInvert = false;

        /* Angle Motor Inverts */
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        // Note, bevel gears should face left (if you're looking at the back)

        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 24;
            public static final double angleOffset = 256.553;
            public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canivoreName);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 26;
            public static final double angleOffset = 91.143;
            public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canivoreName);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 25;
            public static final double angleOffset = 38.760;
            public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canivoreName);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 27;
            public static final double angleOffset = 310.342;
            public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canivoreName);
        }
    }

    public static class DevelopmentBotConstants {
        // See https://github.com/Team364/BaseFalconSwerve for getting these values.

        public static final boolean hasPigeon = false;
        public static final int PIGEON_PORT = 60;

        public static final double trackWidth = 0.5969;
        public static final double wheelBase = 0.5969;
        public static final double wheelDiameter = 0.10033;
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double robotMass = Units.lbsToKilograms(75);

        // robot size
        public static final double widthWithBumpers = Units.inchesToMeters(30 + 3.25 * 2);
        public static final double lengthWithBumpers = Units.inchesToMeters(30 + 3.25 * 2);

        public static final double openLoopRamp = 0.0; // 0.25
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };

        public static final SecondOrderSwerveKinematics swerveKinematics =
                new SecondOrderSwerveKinematics(moduleTranslations);

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Motor Information */
        public static final double driveMotorFreeSpeed = 6380; // RPM of Falcon 500
        public static final double angleMotorFreeSpeed = 6380; // RPM of Falcon 500
        public static final double stallTorque = 4.69;

        /* Angle Motor PID Values */
        public static final double angleKP = 200;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKV = 0.0;
        public static final double angleKS = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKV = 0.0;
        public static final double driveKS = 0.0;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.968230454756032; // meters per second
        public static final double maxAcceleration =
                (stallTorque * driveGearRatio * 4) / (wheelDiameter * robotMass); // 16.52; // meters per second^2
        public static final double maxAngularVelocity = maxSpeed // rad/s
                / Arrays.stream(moduleTranslations)
                        .map(translation -> translation.getNorm())
                        .max(Double::compare)
                        .get();

        /* Calculated Characterization Values */
        public static final double calculatedDriveKS = 0;
        public static final double calculatedDriveKV = (12 / maxSpeed) / GlobalConstants.targetVoltage;
        public static final double calculatedDriveKA = (12 / maxAcceleration) / GlobalConstants.targetVoltage;
        public static final double calculatedAngleKV =
                (12 * 60) / (angleMotorFreeSpeed * Math.toRadians(360 / angleGearRatio));

        /* Precise Driving Mode Values */
        public static final double preciseDrivingModeSpeedMultiplier = 0.2;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Drive Motor Inverts */
        public static final boolean driveMotorInvert = false;

        /* Drive Encoder Inverts */
        public static final boolean driveEncoderInvert = false;

        /* Angle Motor Inverts */
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 24;
            public static final double angleOffset = 60.4 + 180; // 59.9; //241.179;
            //     public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 26;
            public static final double angleOffset = 349.5 - 180; // 348.5; //167.432; // 348.135;
            //     public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 25;
            public static final double angleOffset = 339.9 - 180; // 339.2; //159.609;
            //     public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 27;
            public static final double angleOffset = 85.4 + 180; // 85.4; //268.506; // 94.043;
            //     public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class FieldConstants {

        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth = Units.inchesToMeters(315.5);

        public static final List<AprilTag> aprilTags = List.of(
                new AprilTag(
                        1,
                        new Pose3d(
                                Units.inchesToMeters(610.77),
                                Units.inchesToMeters(42.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                        2,
                        new Pose3d(
                                Units.inchesToMeters(610.77),
                                Units.inchesToMeters(108.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                        3,
                        new Pose3d(
                                Units.inchesToMeters(610.77),
                                Units.inchesToMeters(174.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                        4,
                        new Pose3d(
                                Units.inchesToMeters(636.96),
                                Units.inchesToMeters(265.74),
                                Units.inchesToMeters(27.38),
                                new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                        5,
                        new Pose3d(
                                Units.inchesToMeters(14.25),
                                Units.inchesToMeters(265.74),
                                Units.inchesToMeters(27.38),
                                new Rotation3d())),
                new AprilTag(
                        6,
                        new Pose3d(
                                Units.inchesToMeters(40.45),
                                Units.inchesToMeters(174.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d())),
                new AprilTag(
                        7,
                        new Pose3d(
                                Units.inchesToMeters(40.45),
                                Units.inchesToMeters(108.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d())),
                new AprilTag(
                        8,
                        new Pose3d(
                                Units.inchesToMeters(40.45),
                                Units.inchesToMeters(42.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d())));

        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
                new AprilTagFieldLayout(aprilTags, fieldLength, fieldWidth);

        public static void setAprilTagOrigin() {
            APRIL_TAG_FIELD_LAYOUT.setOrigin(
                    DriverStation.getAlliance() == Optional.of(Alliance.Red)
                            ? OriginPosition.kRedAllianceWallRightSide
                            : OriginPosition.kBlueAllianceWallRightSide);
        }
    }

    public static final class VisionConstants {

        // Currently working, not sure
        // public static final Transform3d limelightRobotToCamera = new Transform3d(
        //         new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(34.25)),
        //         new Rotation3d(0, Math.toRadians(15), Math.PI));

        public static final Transform3d limelightRobotToCamera = new Transform3d(
                new Translation3d(Units.inchesToMeters(-5), Units.inchesToMeters(0), Units.inchesToMeters(34.25)),
                new Rotation3d(0, Math.toRadians(15), Math.PI));

        public static final Transform3d limelightCameraToRobot = limelightRobotToCamera.inverse();
    }
}
