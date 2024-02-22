package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class IntakeAssistCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final VisionSubsystem visionSubsystem;

    Joystick leftJoystick = new Joystick(0);

    public IntakeAssistCommand(SwerveDriveSubsystem swerveDriveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speed = 1;

        double joystickX = leftJoystick.getX();
        double joystickY = leftJoystick.getY();
        double joystickVectorMagnitude = Math.sqrt((Math.pow(joystickX, 2)) + (Math.pow(joystickY, 2)));

        Supplier<Rotation2d> angleOfGamepiece = () -> {if(visionSubsystem.getDetectorInfo().isPresent())
            return Rotation2d.fromDegrees(visionSubsystem.getDetectorInfo().get().tx());
        else
            return new Rotation2d();};

        Rotation2d absoluteTargetAngle = angleOfGamepiece.get();

        double driveJoystickVectorDirection = Math.atan(joystickY / joystickX);
        
        DoubleSupplier strafeRobotRelative = () -> {return Math.cos(driveJoystickVectorDirection) * joystickVectorMagnitude;};

        DoubleSupplier forwardRobotRelative = () -> 0;
            
        swerveDriveSubsystem.setControl(swerveDriveSubsystem.openLoopRobotCentric
            .withVelocityX(forwardRobotRelative.getAsDouble())
            .withVelocityY(strafeRobotRelative.getAsDouble())
            .withRotationalRate(speed));
    }

    @Override
    public void end(boolean interrupted) {

    }

}
