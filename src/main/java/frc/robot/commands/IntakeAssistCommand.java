package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.subsystems.lights.LightsSubsystemB;
import frc.robot.subsystems.lights.LightsSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeAssistCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final VisionSubsystem visionSubsystem;

    DoubleSupplier strafeJoystick;
    DoubleSupplier forwardJoystick;
    DoubleSupplier rotationJoystick;

    PIDController intakingController = new PIDController(1, 0, 0.1);

    public IntakeAssistCommand(SwerveDriveSubsystem swerveDriveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;
        forwardJoystick = forward;
        strafeJoystick = strafe;
        rotationJoystick = rotation;

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        double joystickX = forwardJoystick.getAsDouble();
        double joystickY = strafeJoystick.getAsDouble();
        double joystickVectorMagnitude = Math.sqrt((Math.pow(joystickX, 2)) + (Math.pow(joystickY, 2)));

        double pieceAngle;

        if(visionSubsystem.getDetectorInfo().isPresent()) {
            pieceAngle = visionSubsystem.getDetectorInfo().get().tx();
        } else {
            pieceAngle = 0.0;
        };

        Rotation2d rotation = swerveDriveSubsystem.getRotation();

        double forwardComponent = joystickX * rotation.getCos() + joystickY * rotation.getSin();
        
        double strafeRobotRelative = intakingController.calculate(pieceAngle);
        DoubleSupplier forwardRobotRelative = () -> forwardComponent;
            
        swerveDriveSubsystem.setControl(swerveDriveSubsystem.openLoopRobotCentric
            .withVelocityX(forwardRobotRelative.getAsDouble())
            .withVelocityY(strafeRobotRelative)
            .withRotationalRate(rotationJoystick.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted) {}

}
