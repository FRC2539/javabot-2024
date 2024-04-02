package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lights.LightsSubsystemB;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeAssistCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final VisionSubsystem visionSubsystem;

    DoubleSupplier strafeJoystick;
    DoubleSupplier forwardJoystick;
    DoubleSupplier rotationJoystick;

    PIDController intakingController = new PIDController(0.4, 0, 0.05);

    SlewRateLimiter slewwweerrr = new SlewRateLimiter(40);

    public IntakeAssistCommand(
            SwerveDriveSubsystem swerveDriveSubsystem,
            VisionSubsystem visionSubsystem,
            LightsSubsystemB lights,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotation) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;
        forwardJoystick = forward;
        strafeJoystick = strafe;
        rotationJoystick = rotation;

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        slewwweerrr.reset(0);
    }

    @Override
    public void execute() {

        double joystickX = forwardJoystick.getAsDouble();
        double joystickY = strafeJoystick.getAsDouble();
        double joystickVectorMagnitude = Math.sqrt((Math.pow(joystickX, 2)) + (Math.pow(joystickY, 2)));

        double pieceAngle;
        boolean seesPiece = false;

        if (visionSubsystem.getDetectorInfo().isPresent()) {
            pieceAngle = visionSubsystem.getDetectorInfo().get().ty();
            seesPiece = true;
        } else {
            pieceAngle = 0.0;
            seesPiece = false;
        }

        Rotation2d rotation = swerveDriveSubsystem.getRotation();

        double forwardComponent = joystickX * rotation.getCos() + joystickY * rotation.getSin();

        double strafeRobotRelative = intakingController.calculate(pieceAngle);
        DoubleSupplier forwardRobotRelative = () -> forwardComponent;

        if (seesPiece == true) {
            swerveDriveSubsystem.setControl(swerveDriveSubsystem
                    .openLoopRobotCentric
                    .withVelocityX(forwardRobotRelative.getAsDouble())
                    .withVelocityY(slewwweerrr.calculate(strafeRobotRelative))
                    .withRotationalRate(rotationJoystick.getAsDouble()));
            LightsSubsystemB.LEDSegment.MainStrip.setColor(LightsSubsystemB.green);
        } else {
            swerveDriveSubsystem.setControl(swerveDriveSubsystem
                    .openLoop
                    .withVelocityX(joystickX)
                    .withVelocityY(joystickY)
                    .withRotationalRate(rotationJoystick.getAsDouble()));
        }
    }

    @Override
    public void end(boolean interrupted) {}
}
