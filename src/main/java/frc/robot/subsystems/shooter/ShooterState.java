package frc.robot.subsystems.shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;

public class ShooterState implements Interpolatable<ShooterState> {
    public double topRollerRPM;
    public double bottomRollerRPM;
    public Rotation2d pivotAngle;
    public boolean isVoltageBased;
    public boolean isAngleVoltageBased;

    public ShooterState(double topRollerRPM, double bottomRollerRPM, Rotation2d shooterAngle, boolean isVoltageBased, boolean isAngleVoltageBased) {
        this.topRollerRPM = topRollerRPM;
        this.bottomRollerRPM = bottomRollerRPM;
        this.pivotAngle = shooterAngle;       
        this.isVoltageBased = isVoltageBased;
        this.isAngleVoltageBased = isAngleVoltageBased;
    }

    public ShooterState(double topRollerRPM, double bottomRollerRPM, Rotation2d shooterAngle) {
        this(topRollerRPM, bottomRollerRPM, shooterAngle, false, false);
    }

    public ShooterState(double topRollerRPM, double bottomRollerRPM, double shooterVoltage) {
        this(topRollerRPM, bottomRollerRPM, Rotation2d.fromRotations(shooterVoltage), false, true);
    }

    public static ShooterState fromVoltages(double topRollerVoltage, double bottomRollerVoltage, Rotation2d shooterAngle) {
        return new ShooterState(topRollerVoltage, bottomRollerVoltage, shooterAngle, true, false);
    }

    public static ShooterState fromVoltages(double topRollerVoltage, double bottomRollerVoltage, double shooterVoltage) {
        return new ShooterState(topRollerVoltage, bottomRollerVoltage, Rotation2d.fromRotations(shooterVoltage), true, true);
    }


    public ShooterState() {
        this(0, 0, new Rotation2d(), false, true);
    }

    @Override
    public ShooterState interpolate(ShooterState otherState, double t) {
        return new ShooterState(
                MathUtil.interpolate(this.topRollerRPM, otherState.topRollerRPM, t),
                MathUtil.interpolate(this.bottomRollerRPM, otherState.bottomRollerRPM, t),
                Rotation2d.fromRotations(MathUtil.interpolate(this.pivotAngle.getRotations(), otherState.pivotAngle.getRotations(), t)));
    }
}
