package frc.robot.subsystems.trap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

public class TrapState implements Interpolatable<TrapState> {
    public double topVoltage;
    public double bottomVoltage;
    public double rack;
    public boolean isVoltageBased;

    public TrapState(double topRollerVoltage, double bottomRollerVoltage, double rack, boolean isRackVoltageBased) {
        this.topVoltage = topRollerVoltage;
        this.bottomVoltage = bottomRollerVoltage;
        this.rack = rack;       
        this.isVoltageBased = isRackVoltageBased;
    }

    public TrapState(double topRollerVoltage, double bottomRollerVoltage, double rackPosition) {
        this(topRollerVoltage, bottomRollerVoltage, rackPosition, false);
    }

    public static TrapState fromVoltages(double topRollerVoltage, double bottomRollerVoltage, double rackVoltage) {
        return new TrapState(topRollerVoltage, bottomRollerVoltage, rackVoltage, true);
    }


    public TrapState() {
        this(0, 0, 0.0, true);
    }

    @Override
    public TrapState interpolate(TrapState otherState, double t) {
        return new TrapState(
                MathUtil.interpolate(this.topVoltage, otherState.topVoltage, t),
                MathUtil.interpolate(this.bottomVoltage, otherState.bottomVoltage, t),
                MathUtil.interpolate(this.rack, otherState.rack, t));
    }
}
