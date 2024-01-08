package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interpolation.MovingAverageVelocity;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.lib.swerve.SwerveDriveSignal;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swervedrive.GyroIO.GyroIOInputs;
import java.util.function.DoubleSupplier;

public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator swervePoseEstimator;

    private Pose2d pose = new Pose2d();
    private final MovingAverageVelocity velocityEstimator = new MovingAverageVelocity(3);
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private ChassisSpeeds previousVelocity = new ChassisSpeeds();
    private SwerveDriveSignal driveSignal = new SwerveDriveSignal();

    private double levelingMaxSpeed;

    private boolean isLevelingAuto = false;

    public boolean isRainbow = false;

    private SwerveModule[] modules;

    private GyroIO gyroIO;
    private GyroIOInputs gyroInputs = new GyroIOInputs();

    boolean isCharacterizing = false;

    private LoggedReceiver isSecondOrder;

    // PID controller used for auto-leveling
    // private PIDController tiltController = new PIDController(0.75 / 15, 0, 0.02);
    private PIDController tiltController = new PIDController(0.75 / 12, 0, 0.02);

    // Cardinal command
    private ProfiledPIDController omegaController =
            new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(8, 8));
    private final double maxCardinalVelocity = 4.5;

    private double previousTilt = 0;
    private double tiltRate = 0;
    private DoubleSupplier maxSpeedSupplier = () -> Constants.SwerveConstants.maxSpeed;

    public SwerveDriveSubsystem(GyroIO gyroIO, SwerveModuleIO[] swerveModuleIOs) {
        this.gyroIO = gyroIO;

        modules = new SwerveModule[] {
            new SwerveModule(swerveModuleIOs[0], 0),
            new SwerveModule(swerveModuleIOs[1], 1),
            new SwerveModule(swerveModuleIOs[2], 2),
            new SwerveModule(swerveModuleIOs[3], 3)
        };

        // Initialize the swerve drive pose estimator with access to the module positions.
        swervePoseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.swerveKinematics,
                getGyroRotation(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.01, 0.01, 0.01),
                VecBuilder.fill(0.9, 0.9, 0.9)); // might need to bring these back up (was 0.5)

        // Allow us to toggle on second order kinematics
        isSecondOrder = Logger.tunable("/SwerveDriveSubsystem/isSecondOrder", false);
    }

    public Command driveCommand(
            DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation, boolean isFieldOriented) {
        return run(() -> {
            setVelocity(
                    new ChassisSpeeds(forward.getAsDouble(), strafe.getAsDouble(), rotation.getAsDouble()),
                    isFieldOriented);
        });
    }

    public Command preciseDriveCommand(
            DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation, boolean isFieldOriented) {
        var speedMultiplier = SwerveConstants.preciseDrivingModeSpeedMultiplier;

        return run(() -> {
            setVelocity(
                    new ChassisSpeeds(
                            speedMultiplier * forward.getAsDouble(),
                            speedMultiplier * strafe.getAsDouble(),
                            speedMultiplier * rotation.getAsDouble()),
                    isFieldOriented);
        });
    }

    public Command cardinalCommand(Rotation2d targetAngle, DoubleSupplier forward, DoubleSupplier strafe) {
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        return run(() -> {
                    var rotationCorrection =
                            omegaController.calculate(pose.getRotation().getRadians(), targetAngle.getRadians());

                    setVelocity(
                            new ChassisSpeeds(
                                    forward.getAsDouble(),
                                    strafe.getAsDouble(),
                                    MathUtils.ensureRange(
                                            rotationCorrection, -maxCardinalVelocity, maxCardinalVelocity)),
                            true);
                })
                .beforeStarting(() -> {
                    omegaController.reset(new TrapezoidProfile.State(
                            pose.getRotation().getRadians(), velocity.omegaRadiansPerSecond));
                });
    }

    private class AnyContainer<T> {
        public T thing;

        public AnyContainer(T thing) {
            this.thing = thing;
        }
    }

    public Command levelChargeStationCommand() {
        Timer myFavoriteTimer = new Timer();
        AnyContainer<Double> sketchyBoi = new AnyContainer<Double>(0.5);
        AnyContainer<Boolean> isGoingSlower = new AnyContainer<Boolean>(false);
        return run(() -> {
                    double tilt = getTiltAmountInDegrees();

                    // Negative pitch -> drive forward, Positive pitch -> drive backward

                    Translation2d direction = new Translation2d(
                            1,
                            new Rotation2d(
                                    getNormalVector3d().getX(),
                                    getNormalVector3d().getY()));

                    double speed = tiltController.calculate(tilt, 0);
                    if (speed >= levelingMaxSpeed) speed = levelingMaxSpeed;

                    speed *= (isGoingSlower.thing ? 0.65 : 1);

                    Translation2d finalDirection = direction.times(speed);

                    ChassisSpeeds velocity = new ChassisSpeeds(finalDirection.getX(), finalDirection.getY(), 0);
                    // if (tiltController.atSetpoint()) myFavoriteTimer.restart();

                    sketchyBoi.thing -= 0.02;

                    if (tiltController.atSetpoint()
                            || Math.abs(getTiltRate())
                                    >= Math.toDegrees(0.5)) { // angleRateThresholdReceiver.getDouble())) {
                        sketchyBoi.thing = 0.5;
                        isGoingSlower.thing = true;
                    }

                    if (sketchyBoi.thing > 0) {
                        myFavoriteTimer.start();
                        lock();
                        isRainbow = true;
                    } else {
                        setVelocity(velocity, false);
                        myFavoriteTimer.stop();
                    }
                })
                .beforeStarting(() -> {
                    isGoingSlower.thing = false;
                    myFavoriteTimer.reset();
                    isLevelingAuto = true;
                    sketchyBoi.thing = 0.0;
                    var values = new double[] {0.8 / 15, 0, .01, 8, 0.8};
                    if (values.length < 5) return;
                    tiltController.setPID(values[0], values[1], values[2]);
                    tiltController.setTolerance(values[3]);
                    levelingMaxSpeed = values[4];
                })
                .finallyDo((interrupted) -> {
                    tiltController.reset();
                    isLevelingAuto = false;
                    isRainbow = false;
                });
    }

    public boolean isAutoLeveled() {
        return tiltController.atSetpoint() && isLevelingAuto;
    }

    public void setCustomMaxSpeedSupplier(DoubleSupplier maxSpeedSupplier) {
        this.maxSpeedSupplier = maxSpeedSupplier;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
        swervePoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }

    public void addVisionPoseEstimate(Pose2d pose, double timestamp) {
        swervePoseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public void addVisionPoseEstimate(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        swervePoseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    /**
     * @return The robot relative velocity of the drivetrain
     */
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    public ChassisSpeeds getAcceleration() {
        return new ChassisSpeeds(
                velocity.vxMetersPerSecond - previousVelocity.vxMetersPerSecond,
                velocity.vyMetersPerSecond - previousVelocity.vyMetersPerSecond,
                velocity.omegaRadiansPerSecond - previousVelocity.omegaRadiansPerSecond);
    }

    /**
     * @return The potentially field relative desired velocity of the drivetrain
     */
    public ChassisSpeeds getDesiredVelocity() {
        return (ChassisSpeeds) driveSignal;
    }

    public double getVelocityMagnitude() {
        return Math.sqrt(Math.pow(velocity.vxMetersPerSecond, 2) + Math.pow(velocity.vyMetersPerSecond, 2));
    }

    public Rotation2d getVelocityRotation() {
        return (new Translation2d(velocity.vxMetersPerSecond, velocity.vxMetersPerSecond)).getAngle();
    }

    public ChassisSpeeds getSmoothedVelocity() {
        return velocityEstimator.getAverage();
    }

    public Rotation2d getGyroRotation() {
        return gyroInputs.rotation2d;
    }

    public Rotation2d getRotation() {
        return pose.getRotation();
    }

    public Rotation3d getGyroRotation3d() {
        return gyroInputs.rotation3d;
    }

    public Translation3d getNormalVector3d() {
        return new Translation3d(0, 0, 1).rotateBy(getGyroRotation3d());
    }

    /**is in degrees*/
    public double getTiltAmountInDegrees() {
        return Math.toDegrees(getTiltAmount());
    }

    public double getTiltAmount() {
        return Math.acos(getNormalVector3d().getZ());
    }

    public double getTiltRate() {
        return tiltRate;
    }

    public Rotation2d getTiltDirection() {
        return new Rotation2d(getNormalVector3d().getX(), getNormalVector3d().getY());
    }

    public void setRotation(Rotation2d angle) {
        setPose(new Pose2d(getPose().getX(), getPose().getY(), angle));
    }

    public void zeroRotation() {
        setRotation(new Rotation2d());
    }

    public void setVelocity(ChassisSpeeds velocity, boolean isFieldOriented, boolean isOpenLoop) {
        driveSignal = new SwerveDriveSignal(velocity, isFieldOriented, isOpenLoop);
    }

    public void setVelocity(ChassisSpeeds velocity, boolean isFieldOriented) {
        setVelocity(velocity, isFieldOriented, true);
    }

    public void setVelocity(ChassisSpeeds velocity) {
        setVelocity(velocity, false);
    }

    public void stop() {
        driveSignal = new SwerveDriveSignal();
    }

    public void lock() {
        driveSignal = new SwerveDriveSignal(true);
    }

    private void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeedSupplier.getAsDouble());

        for (SwerveModule module : modules) {
            module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop, isSecondOrder.getBoolean());
        }

        Logger.log("/SwerveDriveSubsystem/Wheel Setpoint", desiredStates[0].speedMetersPerSecond);
    }

    private boolean isDriveSignalStopped(SwerveDriveSignal driveSignal) {
        return driveSignal.vxMetersPerSecond == 0
                && driveSignal.vyMetersPerSecond == 0
                && driveSignal.omegaRadiansPerSecond == 0;
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);

        for (SwerveModule module : modules) {
            module.updateInputs();
        }

        var startTimeMS = Timer.getFPGATimestamp() * 1000;

        Logger.log("/SwerveDriveSubsystem/LoopDuration", Timer.getFPGATimestamp() * 1000 - startTimeMS);

        var tilt = getTiltAmount();
        tiltRate = (tilt - previousTilt) / 0.02;
        previousTilt = tilt;

        // Comment out to play music
        updateOdometry();

        if (isCharacterizing) return;

        updateModules(driveSignal);
        //* endd comment out here */

        updateLogs();
    }

    private void updateOdometry() {
        SwerveModuleState[] moduleStates = getModuleStates();
        SwerveModulePosition[] modulePositions = getModulePositions();

        previousVelocity = velocity;
        velocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(moduleStates);

        // velocityEstimator.add(velocity);



        if (gyroInputs.isActive) {
            pose = swervePoseEstimator.update(getGyroRotation(), modulePositions);
        } else { //TODO: this is a very hacky hack that should be fixed 
            pose = swervePoseEstimator.update(pose.getRotation().plus(Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * 0.05)), modulePositions);
        }
        
    }

    private void updateModules(SwerveDriveSignal driveSignal) {
        ChassisSpeeds chassisVelocity;

        if (driveSignal.isFieldOriented()) {
            chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveSignal.vxMetersPerSecond,
                    driveSignal.vyMetersPerSecond,
                    driveSignal.omegaRadiansPerSecond,
                    getRotation());
        } else {
            chassisVelocity = (ChassisSpeeds) driveSignal;
        }

        SwerveModuleState[] moduleStates =
                Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisVelocity);

        if (driveSignal.isLocked()) {
            // get X for stopping
            moduleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            };

            // Set the angle of each module only
            for (int i = 0; i < moduleStates.length; i++) {
                modules[i].setDesiredAngleOnly(moduleStates[i].angle, true);
            }
        } else {
            setModuleStates(moduleStates, isDriveSignalStopped(driveSignal) ? true : driveSignal.isOpenLoop());
            // setModuleStates(moduleStates, driveSignal.isOpenLoop());
        }
    }

    public void updateLogs() {

        Logger.log("/SwerveDriveSubsystem/Pose", pose);
        // Logger.log("/SwerveDriveSubsystem/Velocity", velocity);
        // Logger.log("/SwerveDriveSubsystem/Desired Velocity", (ChassisSpeeds) driveSignal);

        Logger.log("/SwerveDriveSubsystem/Velocity Magnitude", getVelocityMagnitude());

        Logger.log("/SwerveDriveSubsystem/Wheel Speed", modules[0].getState().speedMetersPerSecond);

        Logger.log("/SwerveDriveSubsystem/Pitch", getGyroRotation3d().getY());
        Logger.log("/SwerveDriveSubsystem/Roll", getGyroRotation3d().getX());
        Logger.log("/SwerveDriveSubsystem/Tilt", getTiltAmountInDegrees());

        Logger.log("/SwerveDriveSubsystem/CANCoder Angles", new double[] {
            modules[0].getCanCoderAngle().getDegrees(),
            modules[1].getCanCoderAngle().getDegrees(),
            modules[2].getCanCoderAngle().getDegrees(),
            modules[3].getCanCoderAngle().getDegrees(),
        });

        Logger.log("/SwerveDriveSubsystem/SwerveModuleStates/Measured", new double[] {
            modules[0].getState().angle.getRadians(), modules[0].getState().speedMetersPerSecond,
            modules[1].getState().angle.getRadians(), modules[1].getState().speedMetersPerSecond,
            modules[2].getState().angle.getRadians(), modules[2].getState().speedMetersPerSecond,
            modules[3].getState().angle.getRadians(), modules[3].getState().speedMetersPerSecond
        });

        Logger.log("/SwerveDriveSubsystem/SwerveModuleStates/Setpoints", new double[] {
            modules[0].getDesiredState().angle.getRadians(), modules[0].getDesiredState().speedMetersPerSecond,
            modules[1].getDesiredState().angle.getRadians(), modules[1].getDesiredState().speedMetersPerSecond,
            modules[2].getDesiredState().angle.getRadians(), modules[2].getDesiredState().speedMetersPerSecond,
            modules[3].getDesiredState().angle.getRadians(), modules[3].getDesiredState().speedMetersPerSecond,
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

        Logger.log("/SwerveDriveSubsystem/Drive Temperatures", getDriveTemperatures());
        Logger.log("/SwerveDriveSubsystem/Angle Temperatures", getAngleTemperatures());

        
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }

    public double[] getDriveTemperatures() {
        return new double[] {
            modules[0].getDriveTemperature(),
            modules[1].getDriveTemperature(),
            modules[2].getDriveTemperature(),
            modules[3].getDriveTemperature()
        };
    }

    public double[] getAngleTemperatures() {
        return new double[] {
            modules[0].getAngleTemperature(),
            modules[1].getAngleTemperature(),
            modules[2].getAngleTemperature(),
            modules[3].getAngleTemperature()
        };
    }
}
