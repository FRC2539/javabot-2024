package frc.robot.subsystems.shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterPosition;

public class ShooterState implements Interpolatable<ShooterState> {
    public double topRollerRPM;
    public double bottomRollerRPM;
    public ShooterPosition shooterPosition;

    public ShooterState(double topRollerRPM, double bottomRollerRPM, ShooterPosition shooterPosition) {
        this.topRollerRPM = topRollerRPM;
        this.bottomRollerRPM = bottomRollerRPM;
        this.shooterPosition = shooterPosition;       
    }

    public ShooterState() {
        this(0, 0, ShooterPosition.FIRST_POSITION);
    }

    @Override
    public ShooterState interpolate(ShooterState otherState, double t) {
        return new ShooterState(
                MathUtil.interpolate(this.topRollerRPM, otherState.topRollerRPM, t),
                MathUtil.interpolate(this.bottomRollerRPM, otherState.bottomRollerRPM, t),
                shooterPosition);
    }
}
