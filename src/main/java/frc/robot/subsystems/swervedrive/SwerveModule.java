package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.swerve.CTREModuleState;
import frc.lib.swerve.SecondOrderSwerveModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveModuleIO.SwerveModuleIOInputs;

public class SwerveModule {

    public int moduleNumber;
    private double lastAngle;

    private SwerveModuleIO swerveModuleIO;

    private SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();

    private SwerveModuleState desiredSetpointState = new SwerveModuleState();

    public SwerveModule(SwerveModuleIO swerveModuleIO, int moduleNumber) {
        this.swerveModuleIO = swerveModuleIO;
        this.moduleNumber = moduleNumber;

        swerveModuleIO.updateInputs(inputs);

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        setDesiredState(desiredState, isOpenLoop, false);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean isSecondOrder) {
        desiredSetpointState = desiredState;

        // Custom optimize command, since default WPILib optimize assumes continuous controller, which CTRE is not
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        if (isOpenLoop) {
            swerveModuleIO.setDesiredVelocityOpenLoop(desiredState.speedMetersPerSecond);
        } else {
            swerveModuleIO.setDesiredVelocityClosedLoop(desiredState.speedMetersPerSecond);
        }

        // Determine the angle to set the module to
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle
                        .getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        lastAngle = angle;

        // Account for the velocity of the angle motor if in second order mode
        if (isSecondOrder && desiredState instanceof SecondOrderSwerveModuleState) {
            swerveModuleIO.setDesiredAngularPositionAndVelocity(
                    angle, ((SecondOrderSwerveModuleState) desiredState).angularVelocityRadiansPerSecond);
        } else {
            swerveModuleIO.setDesiredAngularPosition(angle);
        }
    }

    public void updateInputs() {
        swerveModuleIO.updateInputs(inputs);
    }

    public void setDesiredAngleOnly(Rotation2d desiredAngle, boolean optimized) {
        // Set the module to face forwards
        if (optimized) {
            desiredAngle = CTREModuleState.optimize(new SwerveModuleState(1, desiredAngle), getState().angle).angle;
        }

        swerveModuleIO.setDesiredAngularPosition(desiredAngle.getDegrees());

        lastAngle = 0;

        // Stop the motor to bypass the speed check
        swerveModuleIO.disableDriveMotor();
    }

    public void disableMotors() {
        swerveModuleIO.disableAngleMotor();
        swerveModuleIO.disableDriveMotor();
    }

    public Rotation2d getCanCoderAngle() {
        return inputs.encoderAngle;
    }

    public SwerveModuleState getState() {
        double velocity = inputs.velocity;
        Rotation2d angle = inputs.angularPosition;
        return new SwerveModuleState(velocity, angle);
    }

    public double getAngularVelocity() {
        return inputs.angularVelocity;
    }

    public SwerveModulePosition getPosition() {
        double encoder = inputs.position;
        Rotation2d angle = inputs.angularPosition;
        return new SwerveModulePosition(encoder, angle);
    }

    public SwerveModuleState getDesiredState() {
        return desiredSetpointState;
    }

    public double getDriveTemperature() {
        return inputs.driveTemperature;
    }

    public double getAngleTemperature() {
        return inputs.angleTemperature;
    }

    public double getDriveVoltage() {
        return inputs.driveVoltage;
    }

    public double getAngleVoltage() {
        return inputs.angleVoltage;
    }

    public double getDriveCurrent() {
        return inputs.driveCurrent;
    }

    public double getAngleCurrent() {
        return inputs.angleCurrent;
    }
}
