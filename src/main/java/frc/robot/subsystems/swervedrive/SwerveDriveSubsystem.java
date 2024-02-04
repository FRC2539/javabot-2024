package frc.robot.subsystems.swervedrive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.robot.Constants;
import frc.robot.commands.DriveToPositionCommand;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private SwerveDriveState currentState = new SwerveDriveState();

    public final SwerveRequest.RobotCentric closedLoopRobotCentric = new SwerveRequest.RobotCentric()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public final SwerveRequest.FieldCentric closedLoop = new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public final SwerveRequest.FieldCentric openLoop = new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public final SwerveRequest.ApplyChassisSpeeds stopped = new SwerveRequest.ApplyChassisSpeeds()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        applyConfigs();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        applyConfigs();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), getRotation());
    }

    public double getVelocityMagnitude() {
        ChassisSpeeds currentSpeeds = getRobotRelativeChassisSpeeds();
        return Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public Rotation2d getTilt() {
        return new Rotation2d(Math.acos(getRotation3d().getAxis().get(2,0)));
    }

    private void applyConfigs() {
         AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::seedFieldRelative, // Method to res.et odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (velocity) -> this.setControl(
                    closedLoopRobotCentric
                    .withVelocityX(velocity.vxMetersPerSecond)
                    .withVelocityY(velocity.vyMetersPerSecond)
                    .withRotationalRate(velocity.omegaRadiansPerSecond)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5, 0.0, 0.01), // Translation PID constants
                        new PIDConstants(4, 0.0, 0.01), // Rotation PID constants
                        Constants.SwerveConstants.maxSpeed, // Max module speed, in m/s
                        Constants.SwerveConstants.moduleTranslations[0].getNorm(), // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        registerTelemetry((state) -> currentState = state);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command driveCommand(
            DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        return applyRequest(
                    () ->openLoop.withDeadband(0.0).withRotationalDeadband(0.0)
                    .withVelocityX(forward.getAsDouble())
                    .withVelocityY(strafe.getAsDouble())
                    .withRotationalRate(rotation.getAsDouble())
        );
    }

    public Command aimAtPoseCommand(Supplier<Pose2d> targetPose, DoubleSupplier forward, DoubleSupplier strafes) {
        return directionCommand(() -> targetPose.get().getTranslation().minus(getPose().getTranslation()).getAngle(), forward, strafes,
        new ProfiledPIDController(5, 0, 1, new TrapezoidProfile.Constraints(3.0, 8)));
    }

    public Command cardinalCommand(Rotation2d targetAngle, DoubleSupplier forward, DoubleSupplier strafe) {
        final ProfiledPIDController omegaController = 
            new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(3.0, 8));
        
        return directionCommand(() -> targetAngle, forward, strafe, omegaController);
    }

    public Command directionCommand(Supplier<Rotation2d> targetAngle, DoubleSupplier forward, DoubleSupplier strafe, final ProfiledPIDController omegaController) {
        final double maxCardinalVelocity = omegaController.getConstraints().maxVelocity;

        omegaController.enableContinuousInput(-Math.PI, Math.PI);   

        return run(() -> {
                    var rotationCorrection =
                            omegaController.calculate(getPose().getRotation().getRadians(), targetAngle.get().getRadians());

                    setControl(
                            openLoop
                                .withVelocityX(forward.getAsDouble())
                                .withVelocityY(strafe.getAsDouble())
                                .withRotationalRate(
                                    MathUtils.ensureRange(
                                            rotationCorrection, -maxCardinalVelocity, maxCardinalVelocity)
                            ));
                })
                .beforeStarting(() -> {
                    omegaController.reset(new TrapezoidProfile.State(
                            getRotation().getRadians(), getFieldRelativeChassisSpeeds().omegaRadiansPerSecond));
                });
    }

    public Command pathfindToPoseCommand(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
        3, 3.0,
        Math.PI * 3, Math.PI * 4);


        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
        0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        return Commands.either(
            new DriveToPositionCommand(this, targetPose), 
            pathfindingCommand.andThen(new DriveToPositionCommand(this, targetPose)), 
            () -> targetPose.getTranslation().getDistance(getPose().getTranslation()) < .5);
    }

    public Command driveToPoseCommand(Pose2d targetPose) {
        return new DriveToPositionCommand(this, targetPose);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        logTelemetry(currentState);
    }

    private void logTelemetry(SwerveDriveState state) {
        try {
            Logger.log("/SwerveDriveSubsystem/Pose", state.Pose);
            // Logger.log("/SwerveDriveSubsystem/Velocity", velocity);
            // Logger.log("/SwerveDriveSubsystem/Desired Velocity", (ChassisSpeeds) driveSignal);

            Logger.log("/SwerveDriveSubsystem/Velocity Magnitude", getVelocityMagnitude());

            Logger.log("/SwerveDriveSubsystem/Acceleration Commanded", 0);

            Logger.log("/SwerveDriveSubsystem/Wheel Zero Speed", state.ModuleStates[0].speedMetersPerSecond);

            Rotation3d currentRotation = getRotation3d();

            Logger.log("/SwerveDriveSubsystem/Pitch", currentRotation.getY());
            Logger.log("/SwerveDriveSubsystem/Roll", currentRotation.getX());
            Logger.log("/SwerveDriveSubsystem/Tilt", getTilt().getRadians());

            Logger.log("/SwerveDriveSubsystem/CANCoder Angles", new double[] {
                state.ModuleStates[0].angle.getDegrees(),
                state.ModuleStates[1].angle.getDegrees(),
                state.ModuleStates[2].angle.getDegrees(),
                state.ModuleStates[3].angle.getDegrees(),
            });

            // // Raw In This Case means the offset is not applied
            // Logger.log("/SwerveDriveSubsystem/Raw CANCoder Angles", new double[] {
            //     state.ModuleStates[0].,
            //     state.ModuleStates[0].angle.getDegrees(),
            //     modules[2].getRawCanCoderAngle().getDegrees(),
            //     modules[3].getRawCanCoderAngle().getDegrees(),
            // });

            Logger.log("/SwerveDriveSubsystem/SwerveModuleStates/Measured", new double[] {
                state.ModuleStates[0].angle.getRadians(), state.ModuleStates[0].speedMetersPerSecond,
                state.ModuleStates[1].angle.getRadians(), state.ModuleStates[1].speedMetersPerSecond,
                state.ModuleStates[2].angle.getRadians(), state.ModuleStates[2].speedMetersPerSecond,
                state.ModuleStates[3].angle.getRadians(), state.ModuleStates[3].speedMetersPerSecond
            });

            Logger.log("/SwerveDriveSubsystem/SwerveModuleStates/Setpoints", new double[] {
                state.ModuleTargets[0].angle.getRadians(), state.ModuleTargets[0].speedMetersPerSecond,
                state.ModuleTargets[1].angle.getRadians(), state.ModuleTargets[1].speedMetersPerSecond,
                state.ModuleTargets[2].angle.getRadians(), state.ModuleTargets[2].speedMetersPerSecond,
                state.ModuleTargets[3].angle.getRadians(), state.ModuleTargets[3].speedMetersPerSecond
            });

            // Logger.log("/SwerveDriveSubsystem/Wheel Angles", new double[] {
            //     modules[0].getPosition().angle.getDegrees(),
            //     modules[1].getPosition().angle.getDegrees(),
            //     modules[2].getPosition().angle.getDegrees(),
            //     modules[3].getPosition().angle.getDegrees()
            // });

            // Logger.log("/SwerveDriveSubsystem/Angle Wheel Amps", new double[] {
            //     modules[0].getAngleCurrent(),
            //     modules[1].getAngleCurrent(),
            //     modules[2].getAngleCurrent(),
            //     modules[3].getAngleCurrent()
            // });

            // Logger.log("/SwerveDriveSubsystem/Angle Wheel Volts", new double[] {
            //     modules[0].getAngleVoltage(),
            //     modules[1].getAngleVoltage(),
            //     modules[2].getAngleVoltage(),
            //     modules[3].getAngleVoltage()
            // });

            Logger.log("/SwerveDriveSubsystem/Drive Temperatures", new double[] {          
                getModule(0).getDriveMotor().getDeviceTemp().getValue(),
                getModule(1).getDriveMotor().getDeviceTemp().getValue(),
                getModule(2).getDriveMotor().getDeviceTemp().getValue(),
                getModule(3).getDriveMotor().getDeviceTemp().getValue()
            });
            Logger.log("/SwerveDriveSubsystem/Angle Temperatures", new double[] {          
                getModule(0).getSteerMotor().getDeviceTemp().getValue(),
                getModule(1).getSteerMotor().getDeviceTemp().getValue(),
                getModule(2).getSteerMotor().getDeviceTemp().getValue(),
                getModule(3).getSteerMotor().getDeviceTemp().getValue()
            });
        } catch (Exception e) {
            System.err.print(e);
        }
    }
}
