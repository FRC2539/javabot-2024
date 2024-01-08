package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {
    public void updateInputs(SwerveModuleIOInputs inputs);

    public class SwerveModuleIOInputs {
        public double angularVelocity = 0;
        public double velocity = 0;
        public Rotation2d angularPosition = new Rotation2d();
        public double position = 0;
        public Rotation2d encoderAngle = new Rotation2d();

        public double driveTemperature = 0;
        public double angleTemperature = 0;
        public double driveVoltage = 0;
        public double angleVoltage = 0;
        public double driveCurrent = 0;
        public double angleCurrent = 0;
    }

    public void setDesiredVelocityOpenLoop(double velocity);

    public void setDesiredVelocityClosedLoop(double velocity);

    public void setDesiredAngularPosition(double angularPosition);

    public void setDesiredAngularPositionAndVelocity(double angularPosition, double angularVelocity);

    public void disableDriveMotor();

    public void disableAngleMotor();
}
