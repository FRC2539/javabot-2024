package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.interpolation.InterpolatableDouble;
import frc.lib.interpolation.InterpolatingMap;

import java.util.Arrays;

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

    public static final class IntakeConstants {
        public static final int chamberMotorPort = 15;
        public static final int rollerMotorPort = 14;
        public static final int chamberSensorPort = 0;
        public static final int rollerSensorPort = 1;
    }

    public static final class ShooterConstants {
        public static final int bottomShooterPort = 8;
        public static final int topShooterPort = 9;
        public static final int pivotPort = 13;

        public static final double gearRatioRoller = 1;
        public static final double momentOfInertiaRoller = 1;

        public static final double gearRatioPivot = 1;
        public static final double momentOfInertiaPivot = 1;
        public static final double comPivot = 1;
        public static final double massPivot = 1;

        public static final int talonFXPort = 1;

        public static final InterpolatingMap<InterpolatableDouble> topRollerMap() {
            var map = new InterpolatingMap<InterpolatableDouble>();
            map.put(0, new InterpolatableDouble(0.0));
            map.put(5, new InterpolatableDouble(1.0));
            map.put(15, new InterpolatableDouble(10.0));
            map.put(100, new InterpolatableDouble(10.0));

            return map;
        }

        public static final InterpolatingMap<InterpolatableDouble> bottomRollerMap() {
            var map = new InterpolatingMap<InterpolatableDouble>();
            map.put(0, new InterpolatableDouble(0.0));
            map.put(5, new InterpolatableDouble(1.0));
            map.put(15, new InterpolatableDouble(10.0));
            map.put(100, new InterpolatableDouble(10.0));

            return map;
        }

        public static final InterpolatingMap<InterpolatableDouble> shooterAngleMap() {
            var map = new InterpolatingMap<InterpolatableDouble>();
            map.put(0, new InterpolatableDouble(0.0));
            map.put(5, new InterpolatableDouble(.5));
            map.put(15, new InterpolatableDouble(.75));
            map.put(100, new InterpolatableDouble(1));

            return map;
        }
    }

    public static class SwerveConstants {
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
        public static final double driveKV = 0.11;
        public static final double driveKS = 0.12;
        public static final double driveKA = 0.0;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.968230454756032; // meters per second
        public static final double maxAcceleration =
                (stallTorque * driveGearRatio * 4) / (wheelDiameter * robotMass); // 16.52; // meters per second^2
        public static final double maxAngularVelocity = maxSpeed // rad/s
                / Arrays.stream(moduleTranslations)
                        .map(translation -> translation.getNorm())
                        .max(Double::compare)
                        .get();

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

        // To tune the angle offset, simply align the wheels so that if they spun, the robot would drive directly forward. 
        // Make sure that if you are looking at the front of the robot, the bevels are on the left.

        /* Front Left Module - Module 0 */
    }

    public static final class FieldConstants {

        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth = Units.inchesToMeters(315.5);

        public static AprilTagFieldLayout aprilTagFieldLayout = getFieldLayout();

        private static AprilTagFieldLayout getFieldLayout() {
                try {
                        return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
                } catch (Exception e) {
                        // TODO: Fix this trash
                        return null;
                }
        }

        public static Pose2d getSpeakerPose() {
            return aprilTagFieldLayout.getTagPose(getSpeakerTag()).get().toPose2d();
        }

        public static int getSpeakerTag() {
            return isBlue() ? 7 : 4;
        }

        public static int getAmpAprilTag() {
            return isBlue() ? 6 : 5;
        }

        public static boolean isBlue() {
            return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
        }

        public static Pose2d conditionallyFlipPose(Pose2d pose) {
            return isBlue() ? pose : GeometryUtil.flipFieldPose(pose);
        }
    }


    public static final class VisionConstants {

        // Currently working, not sure
        // public static final Transform3d limelightRobotToCamera = new Transform3d(
        //         new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(34.25)),
        //         new Rotation3d(0, Math.toRadians(15), Math.PI));
        public static final Transform3d robotToLeftCamera = new Transform3d(0,0,0, new Rotation3d(0,0,0)).plus(new Transform3d(Units.inchesToMeters(54),0, Units.inchesToMeters(54), new Rotation3d(0,0,Math.PI)).plus(new Transform3d(
            1.762,
            0.448,
            0.141,
            new Rotation3d(new Quaternion(0.196,-0.220,0.078,-0.952))
        ).inverse()));
        public static final Transform3d robotToRightCamera = new Transform3d(0,0,0, new Rotation3d(0,0,0)).plus(new Transform3d(Units.inchesToMeters(54),0, Units.inchesToMeters(54), new Rotation3d(0,0,Math.PI)).plus(new Transform3d(
            1.630,
            -0.492,
            0.315,
            new Rotation3d(new Quaternion(-0.190,-0.174,-0.037,-0.966))
        ).inverse()));

        public static final Transform3d limelightRobotToCamera = new Transform3d(
                new Translation3d(Units.inchesToMeters(-5), Units.inchesToMeters(0), Units.inchesToMeters(34.25)),
                new Rotation3d(0, Math.toRadians(15), Math.PI));

        public static final Transform3d limelightCameraToRobot = limelightRobotToCamera.inverse();
    }
}
