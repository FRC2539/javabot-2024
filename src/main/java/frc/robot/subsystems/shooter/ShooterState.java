package frc.robot.subsystems.shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;

public class ShooterState implements Interpolatable<ShooterState> {
    public double topRollerRPM;
    public double bottomRollerRPM;
    public Rotation2d pivotAngle;
    public boolean isVoltageBased;

    public ShooterState(double topRollerRPM, double bottomRollerRPM, Rotation2d shooterAngle, boolean isVoltageBased) {
        this.topRollerRPM = topRollerRPM;
        this.bottomRollerRPM = bottomRollerRPM;
        this.pivotAngle = shooterAngle;       
        this.isVoltageBased = isVoltageBased;
    }

    public ShooterState(double topRollerRPM, double bottomRollerRPM, Rotation2d shooterAngle) {
        this(topRollerRPM, bottomRollerRPM, shooterAngle, false);
    }

    public static ShooterState fromVoltages(double topRollerVoltage, double bottomRollerVoltage, Rotation2d shooterAngle) {
        return new ShooterState(topRollerVoltage, bottomRollerVoltage, shooterAngle, true);
    }


    public ShooterState() {
        this(0, 0, new Rotation2d(), false);
    }

    @Override
    public ShooterState interpolate(ShooterState otherState, double t) {
        return new ShooterState(
                MathUtil.interpolate(this.topRollerRPM, otherState.topRollerRPM, t),
                MathUtil.interpolate(this.bottomRollerRPM, otherState.bottomRollerRPM, t),
                Rotation2d.fromRotations(MathUtil.interpolate(this.pivotAngle.getRotations(), otherState.pivotAngle.getRotations(), t)));
    }
}
