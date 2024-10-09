package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ChoreoTrajectoryFollower {

    // This is similar to the usual choreoSwerveCommand but it does not sent outputTrajectoryStates to the drivetrain
    // itself. Allows for feedforward control.
    public static Command customChoreoSwerveCommand(
            ChoreoTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            BiConsumer<Pose2d, ChoreoTrajectoryState> outputTrajectoryState,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            BooleanSupplier mirrorTrajectory,
            Subsystem... requirements) {
        var timer = new Timer();
        return new FunctionalCommand(
                timer::restart,
                () -> {
                    outputTrajectoryState.accept(
                            poseSupplier.get(), trajectory.sample(timer.get(), mirrorTrajectory.getAsBoolean()));
                },
                (interrupted) -> {
                    timer.stop();
                    if (interrupted) {
                        outputChassisSpeeds.accept(new ChassisSpeeds());
                    } else {
                        outputChassisSpeeds.accept(trajectory.getFinalState().getChassisSpeeds());
                    }
                },
                () -> timer.hasElapsed(trajectory.getTotalTime()),
                requirements);
    }

    public static Command customChoreoSwerveAuto(String choreoTrajectoryName, String pathPlannerAutoName) {
        var trajectories = Choreo.getTrajectoryGroup(choreoTrajectoryName);
        var auto = PathPlannerAuto.getPathGroupFromAutoFile(pathPlannerAutoName);
        auto.get(0).getEventMarkers();

        return new PathPlannerAuto(pathPlannerAutoName);
    }
}
