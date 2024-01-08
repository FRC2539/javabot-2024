package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private double angularVelocity;
    private double velocity;
    private double angularPosition;
    private double position;

    public SwerveModuleIOSim() {};

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.angularVelocity = angularVelocity;
        inputs.velocity = velocity;
        inputs.angularPosition = new Rotation2d(angularPosition);
        inputs.position = position;
        inputs.encoderAngle = new Rotation2d(angularPosition);

        inputs.driveTemperature = 50;
        inputs.angleTemperature = 50;
        inputs.driveVoltage = 0;
        inputs.angleVoltage = 0;
        inputs.driveCurrent = 0;
        inputs.angleCurrent = 0;

        angularPosition += angularVelocity * Robot.kDefaultPeriod;
        position += velocity * Robot.kDefaultPeriod;
    };

    public void setDesiredVelocityOpenLoop(double velocity) {
        this.velocity = velocity;
    };

    public void setDesiredVelocityClosedLoop(double velocity) {
        this.velocity = velocity;
    };

    public void setDesiredAngularPosition(double angularPosition) {
        this.angularPosition = Math.toRadians(angularPosition);
        angularVelocity = 0;
    };

    public void setDesiredAngularPositionAndVelocity(double angularPosition, double angularVelocity) {
        this.angularPosition = Math.toRadians(angularPosition);
        this.angularVelocity = angularVelocity;
    };

    public void disableDriveMotor() {
        velocity = 0;
    };

    public void disableAngleMotor() {
        angularVelocity = 0;
    };
}
