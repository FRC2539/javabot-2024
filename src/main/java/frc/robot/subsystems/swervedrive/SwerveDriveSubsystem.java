package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.lib.math.MathUtils.AnyContainer;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.TunerConstants;
import frc.robot.commands.DriveToPositionCommand;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private SwerveDriveState currentState = new SwerveDriveState();

    private Optional<Rotation2d> autoRotationOverride = Optional.empty();

    public Optional<Double> autoRotationVelocityOverride = Optional.empty();

    public Function<Double, Double> autoStrafeOverrideSupplier = (Double x) -> x;

    public final SwerveRequest.RobotCentric closedLoopRobotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public final SwerveRequest.FieldCentric closedLoop = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public final SwerveRequest.FieldCentric openLoop = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public final SwerveRequest.RobotCentric openLoopRobotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public final SwerveRequest.ApplyChassisSpeeds stopped = new SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
            
    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public SwerveDriveSubsystem(
            SwerveDrivetrainConstants driveTrainConstants,
            double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
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
        return new Rotation2d(Math.acos(getRotation3d().getAxis().get(2, 0)));
    }

    private void applyConfigs() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::seedFieldRelative, // Method to res.et odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (velocity) -> {
                    // var correctedVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity, new
                    // Rotation2d(getRobotRelativeChassisSpeeds().omegaRadiansPerSecond *
                    // SwerveConstants.angularVelocityCoefficient));
                    this.setControl(closedLoopRobotCentric
                            .withVelocityX(velocity.vxMetersPerSecond)
                            .withVelocityY(autoStrafeOverrideSupplier.apply(velocity.vyMetersPerSecond))
                            .withRotationalRate(
                                    autoRotationVelocityOverride.isEmpty()
                                            ? velocity.omegaRadiansPerSecond
                                            : autoRotationVelocityOverride.get()));
                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                        // Constants class
                        new PIDConstants(3, 0.0, 0.05), // Translation PID constants
                        new PIDConstants(2, 0.0, 0.05), // Rotation PID constants
                        TunerConstants.kSpeedAt12VoltsMps, // Max module speed, in m/s
                        Constants.SwerveConstants.moduleTranslations[0]
                                .getNorm(), // Drive base radius in meters. Distance from robot center to furthest
                        // module.
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

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationOverride);

        registerTelemetry((state) -> currentState = state);

        for (SwerveModule i : Modules) {
            TalonFX steerMotor = i.getSteerMotor();
            TalonFX driveMotor = i.getDriveMotor();
            CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
            steerMotor.getConfigurator().refresh(currentLimitsConfigs);
            currentLimitsConfigs.StatorCurrentLimit = 100;
            currentLimitsConfigs.StatorCurrentLimitEnable = true;
            currentLimitsConfigs.SupplyCurrentLimit = 80;
            currentLimitsConfigs.SupplyTimeThreshold = 0.4;
            currentLimitsConfigs.SupplyCurrentLimitEnable = true;
            steerMotor.getConfigurator().apply(currentLimitsConfigs);
            TorqueCurrentConfigs torqueCurrentConfigs = new TorqueCurrentConfigs();
            steerMotor.getConfigurator().refresh(torqueCurrentConfigs);
            torqueCurrentConfigs.PeakForwardTorqueCurrent = 80;
            torqueCurrentConfigs.PeakReverseTorqueCurrent = -80;
            steerMotor.getConfigurator().apply(torqueCurrentConfigs);

            double rampTime = 0.0;

            OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
            openLoopRampsConfigs.VoltageOpenLoopRampPeriod = rampTime;
            openLoopRampsConfigs.TorqueOpenLoopRampPeriod = rampTime;
            openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = rampTime;
            ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
            closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = rampTime;
            closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = rampTime;
            closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = rampTime;
            steerMotor.getConfigurator().apply(closedLoopRampsConfigs);
            steerMotor.getConfigurator().apply(openLoopRampsConfigs);

            driveMotor.getConfigurator().apply(closedLoopRampsConfigs);
            driveMotor.getConfigurator().apply(openLoopRampsConfigs);
        }
    }

    public Optional<Rotation2d> getRotationOverride() {
        return autoRotationOverride;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command driveCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        // var correctedVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(new
        // ChassisSpeeds(forward.getAsDouble(),strafe.getAsDouble(),rotation.getAsDouble()), new
        // Rotation2d(getRobotRelativeChassisSpeeds().omegaRadiansPerSecond *
        // SwerveConstants.angularVelocityCoefficient));
        return applyRequest(() -> {
            return openLoop.withDeadband(0.0)
                    .withRotationalDeadband(0.0)
                    .withVelocityX(forward.getAsDouble())
                    .withVelocityY(strafe.getAsDouble())
                    .withRotationalRate(rotation.getAsDouble());
        });
    }

    public Command aimAtPoseCommand(Supplier<Pose2d> targetPose, DoubleSupplier forward, DoubleSupplier strafes) {
        return directionCommand(
                () -> targetPose
                        .get()
                        .getTranslation()
                        .minus(getPose().getTranslation())
                        .getAngle(),
                forward,
                strafes,
                new ProfiledPIDController(3, 0,.5, new TrapezoidProfile.Constraints(3.0, 8)));
    }

    public Command cardinalCommand(Rotation2d targetAngle, DoubleSupplier forward, DoubleSupplier strafe) {
        final ProfiledPIDController omegaController =
                new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(3.0, 8));

        return directionCommand(
                () -> targetAngle, forward, strafe, new PIDController(4, 0, .2), false); // omegaController);
    }

    public Command cardinalCommand(Supplier<Rotation2d> targetAngle, DoubleSupplier forward, DoubleSupplier strafe) {
        final ProfiledPIDController omegaController =
                new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(3.0, 8));

        return directionCommand(
                targetAngle, forward, strafe, new PIDController(4, 0, .2), false); // omegaController);
    }

    private double directionCommandErrorRadiansRotation = 0;
    private double directionCommandErrorRaidansVelocity = 0;
    private boolean directionCommandIsRunning = false;

    public Command directionCommand(
            Supplier<Rotation2d> targetAngle,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            final ProfiledPIDController omegaController) {
        return directionCommand(targetAngle, forward, strafe, omegaController, false);
    }

    public Command directionCommand(
            Supplier<Rotation2d> targetAngle,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            final ProfiledPIDController omegaController,
            boolean closedLoop) {
        final double maxCardinalVelocity = omegaController.getConstraints().maxVelocity;

        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        return run(() -> {
                    var currentRotation = getPose().getRotation().getRadians();
                    var currentTarget = targetAngle.get().getRadians();
                    var currentVelocity = getRobotRelativeChassisSpeeds().omegaRadiansPerSecond;

                    var rotationCorrection = omegaController.calculate(currentRotation, currentTarget);

                    setControl(
                            closedLoop
                                    ? closedLoopRobotCentric
                                            .withVelocityX(forward.getAsDouble())
                                            .withVelocityY(strafe.getAsDouble())
                                            .withRotationalRate(MathUtils.ensureRange(
                                                    rotationCorrection + omegaController.getSetpoint().velocity,
                                                    -maxCardinalVelocity,
                                                    maxCardinalVelocity))
                                    : openLoop.withVelocityX(forward.getAsDouble())
                                            .withVelocityY(strafe.getAsDouble())
                                            .withRotationalRate(MathUtils.ensureRange(
                                                    rotationCorrection + omegaController.getSetpoint().velocity,
                                                    -maxCardinalVelocity,
                                                    maxCardinalVelocity)));

                    directionCommandErrorRadiansRotation = currentRotation - currentTarget;
                    directionCommandErrorRaidansVelocity = currentVelocity - 0;
                    directionCommandIsRunning = true;
                })
                .beforeStarting(() -> {
                    omegaController.reset(new TrapezoidProfile.State(
                            getRotation().getRadians(), getFieldRelativeChassisSpeeds().omegaRadiansPerSecond));
                })
                .finallyDo(() -> {
                    directionCommandIsRunning = false;
                });
    }

    public Command directionCommand(
            Supplier<Rotation2d> targetAngle,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            final PIDController omegaController,
            boolean closedLoop) {
        final double maxCardinalVelocity = 4.5;

        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        return run(() -> {
                    var currentRotation = getPose().getRotation().getRadians();
                    var currentTarget = targetAngle.get().getRadians();
                    var currentVelocity = getRobotRelativeChassisSpeeds().omegaRadiansPerSecond;

                    var rotationCorrection = omegaController.calculate(currentRotation, currentTarget);

                    setControl(
                            closedLoop
                                    ? closedLoopRobotCentric
                                            .withVelocityX(forward.getAsDouble())
                                            .withVelocityY(strafe.getAsDouble())
                                            .withRotationalRate(MathUtils.ensureRange(
                                                    rotationCorrection, -maxCardinalVelocity, maxCardinalVelocity))
                                    : openLoop.withVelocityX(forward.getAsDouble())
                                            .withVelocityY(strafe.getAsDouble())
                                            .withRotationalRate(MathUtils.ensureRange(
                                                    rotationCorrection, -maxCardinalVelocity, maxCardinalVelocity)));

                    directionCommandErrorRadiansRotation = currentRotation - currentTarget;
                    directionCommandErrorRaidansVelocity = currentVelocity - 0;
                    directionCommandIsRunning = true;
                })
                .beforeStarting(() -> {})
                .finallyDo(() -> {
                    directionCommandIsRunning = false;
                });
    }

    public Command directionCommandAutoVelocity(Supplier<Rotation2d> targetAngle, PIDController pidController) {
        // Uses Commands because technically this command has no requirements.
        return Commands.run(() -> {
                    var currentRotation = getPose().getRotation().getRadians();
                    var currentTarget = targetAngle.get().getRadians();
                    var currentVelocity = getRobotRelativeChassisSpeeds().omegaRadiansPerSecond;

                    var output = pidController.calculate(currentRotation, currentTarget);

                    autoRotationVelocityOverride = Optional.of(output);

                    directionCommandErrorRadiansRotation = currentRotation - currentTarget;
                    directionCommandErrorRaidansVelocity = currentVelocity - 0;
                    directionCommandIsRunning = true;
                })
                .finallyDo(() -> {
                    directionCommandIsRunning = false;
                    autoRotationVelocityOverride = Optional.empty();
                });
    }

    public Command directionCommandAuto(Supplier<Rotation2d> targetAngle) {
        // Uses Commands because technically this command has no requirements.
        return Commands.run(() -> {
                    var currentRotation = getPose().getRotation().getRadians();
                    var currentTarget = targetAngle.get().getRadians();
                    var currentVelocity = getRobotRelativeChassisSpeeds().omegaRadiansPerSecond;

                    autoRotationOverride = Optional.of(targetAngle.get());

                    directionCommandErrorRadiansRotation = currentRotation - currentTarget;
                    directionCommandErrorRaidansVelocity = currentVelocity - 0;
                    directionCommandIsRunning = true;
                })
                .finallyDo(() -> {
                    directionCommandIsRunning = false;
                    autoRotationOverride = Optional.empty();
                });
    }

    public void setAutoRotationOverride(Rotation2d override) {
        autoRotationOverride = Optional.of(override);
    }

    public void clearAutoRotationOverride() {
        autoRotationOverride = Optional.empty();
    }

    public boolean isAtDirectionCommand(double angularTolerance, double velocityTolerance) {
        // the acos(cos(x)) bascially figures out the net error accounting for looping
        boolean result = Math.acos(Math.cos(directionCommandErrorRadiansRotation)) <= angularTolerance
                && Math.abs(directionCommandErrorRaidansVelocity) <= velocityTolerance
                && directionCommandIsRunning;

        Logger.log("/SwerveDriveSubsystem/DirectionCommand/AtAngle", result);

        return result;
    }

    public Command drivePredictionCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {

        Supplier<Command> test = () -> {
            final double aheadPlanningDistance = 2;

            Translation2d direction = new Translation2d(forward.getAsDouble(), strafe.getAsDouble());

            Translation2d normDirection = direction.div(direction.getNorm()).times(aheadPlanningDistance);

            Pose2d targetPose = new Pose2d(
                    getPose().getTranslation().plus(normDirection),
                    getRotation().plus(new Rotation2d(rotation.getAsDouble() * 1)));

            PathConstraints constraints = new PathConstraints(5, 5.0, Math.PI * 3, Math.PI * 4);

            Command pathfindingCommand = AutoBuilder.pathfindToPose(
                    targetPose,
                    constraints,
                    0, // Goal end velocity in meters/sec
                    0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting
                    // to rotate.
                    );

            return Commands.either(
                    new DriveToPositionCommand(this, targetPose),
                    pathfindingCommand.andThen(new DriveToPositionCommand(this, targetPose)),
                    () -> targetPose.getTranslation().getDistance(getPose().getTranslation()) < .5);
        };

        AnyContainer<Command> runnableComand = new AnyContainer<Command>(run(() -> {}));

        return Commands.runOnce(() -> {
                    runnableComand.thing = test.get();
                    runnableComand.thing.schedule();
                })
                .andThen(Commands.waitSeconds(.25))
                .repeatedly()
                .finallyDo(() -> runnableComand.thing.cancel());
    }

    public Command pathfindToPoseCommand(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(3, 3.0, Math.PI * 3, Math.PI * 4);

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to
                // rotate.
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

    private CircularBuffer<Pose2d> previousSwervePoses = new CircularBuffer<Pose2d>(50);
    private CircularBuffer<Double> previousSwervePosesTimestamps = new CircularBuffer<Double>(50);

    public Pose2d getPoseAtTimestamp(double timestamp) {
        for (int i = 0; i < previousSwervePosesTimestamps.size(); i++) {
            if (previousSwervePosesTimestamps.get(i) <= timestamp) {
                return previousSwervePoses.get(i);
            }
        }
        return previousSwervePoses.getLast();
    }

    @Override
    public void periodic() {
        previousSwervePoses.addFirst(currentState.Pose);
        previousSwervePosesTimestamps.addFirst(Timer.getFPGATimestamp());
        logTelemetry(currentState);
    }

    private void logTelemetry(SwerveDriveState state) {
        try {
            Logger.log("/SwerveDriveSubsystem/Pose", state.Pose);
            Logger.log("/DemoLogging/robotPose", FieldConstants.transformPoseToDemoSpace(new Pose3d(state.Pose)), true);
            Logger.log("/SwerveDriveSubsystem/DelayedPose", getPoseAtTimestamp(Timer.getFPGATimestamp() - .5));
            Logger.log("/SwerveDriveSubsystem/DelayedTimestamp", previousSwervePosesTimestamps.getLast());
            Logger.log("/SwerveDriveSubsystem/Timestamp", Timer.getFPGATimestamp());
            // Logger.log("/SwerveDriveSubsystem/Velocity", velocity);
            // Logger.log("/SwerveDriveSubsystem/Desired Velocity", (ChassisSpeeds) driveSignal);

            Logger.log("/SwerveDriveSubsystem/Velocity Magnitude", getVelocityMagnitude());

            Logger.log("/SwerveDriveSubsystem/Acceleration Commanded", 0);

            Logger.log("/SwerveDriveSubsystem/Wheel Zero Speed", state.ModuleStates[0].speedMetersPerSecond);
            Logger.log(
                    "/SwerveDriveSubsystem/Wheel Zero Voltage",
                    getModule(0).getDriveMotor().getMotorVoltage().getValue());

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

            Logger.log("/SwerveDriveSubsystem/Drive Wheel Amps", new double[] {
                getModule(0).getDriveMotor().getStatorCurrent().getValue(),
                getModule(1).getDriveMotor().getStatorCurrent().getValue(),
                getModule(2).getDriveMotor().getStatorCurrent().getValue(),
                getModule(3).getDriveMotor().getStatorCurrent().getValue()
            });

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

        Logger.log(
                "/SwerveDriveSubsystem/DirectionCommand/DirectionError",
                Math.acos(Math.cos(directionCommandErrorRadiansRotation)));
        Logger.log("/SwerveDriveSubsystem/DirectionCommand/VelocityError", directionCommandErrorRaidansVelocity);
        Logger.log("/SwerveDriveSubsystem/DirectionCommand/IsRunning", directionCommandIsRunning);
    }
}
