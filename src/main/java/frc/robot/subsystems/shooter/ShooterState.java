package frc.robot.subsystems.shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

public class ShooterState implements Interpolatable<ShooterState> {
    public double topRollerRPM;
    public double bottomRollerRPM;
    public double pivotAngle;

    public ShooterState(double topRollerRPM, double bottomRollerRPM, double shooterAngle) {
        this.topRollerRPM = topRollerRPM;
        this.bottomRollerRPM = bottomRollerRPM;
        this.pivotAngle = shooterAngle;       
    }

    public ShooterState() {
        this(0, 0, 0);
    }

    @Override
    public ShooterState interpolate(ShooterState otherState, double t) {
        return new ShooterState(
                MathUtil.interpolate(this.topRollerRPM, otherState.topRollerRPM, t),
                MathUtil.interpolate(this.bottomRollerRPM, otherState.bottomRollerRPM, t),
                MathUtil.interpolate(this.pivotAngle, otherState.pivotAngle, t));
    }
}
