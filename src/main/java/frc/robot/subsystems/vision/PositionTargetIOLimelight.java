package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;

public class PositionTargetIOLimelight implements PositionTargetIO {
    LoggedReceiver tx = Logger.receive("/limelight/tx", 0.0d);
    LoggedReceiver ty = Logger.receive("/limelight/ty", 0.0d);
    LoggedReceiver tl = Logger.receive("/limelight/cl", 0.0d);
    LoggedReceiver tv = Logger.receive("/limelight/tv", 0);

    public PositionTargetIOLimelight() {
        
    }

    public Optional<PositionTargetIOInputs> updateInputs() {
        if (tv.getInteger() != 1) {
            return Optional.empty();
        }

        var myThingy = new PositionTargetIOInputs();
        myThingy.pitch = ty.getDouble();
        myThingy.yaw = tx.getDouble();
        myThingy.timestamp = tl.getDouble() / 1000 + Timer.getFPGATimestamp();

        return Optional.of(myThingy);
    }

    public String getName() {
        return "limelight";
    }
}
