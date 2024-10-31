package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.vision.LimelightHelpers;

public class PositionTargetIOLimelight implements PositionTargetIO {
    LoggedReceiver tx;;
    LoggedReceiver ty;
    LoggedReceiver tl;
    LoggedReceiver tv;

    public PositionTargetIOLimelight(String limelightName) {
        tx = Logger.receive("/" + limelightName + "/tx", 0.0d);
        ty = Logger.receive("/" + limelightName + "/ty", 0.0d);
        tl = Logger.receive("/" +limelightName + "/cl", 0.0d);
        tv = Logger.receive("/" +limelightName + "/tv", 0);
        LimelightHelpers.setPipelineIndex(limelightName, 1);
    }

    public Optional<PositionTargetIOInputs> updateInputs() {
        try {
            if (tv.getInteger() != 1) {
                return Optional.empty();
            }

            var myThingy = new PositionTargetIOInputs();
            myThingy.pitch = ty.getDouble();
            myThingy.yaw = tx.getDouble();
            myThingy.timestamp = tl.getDouble() / 1000 + Timer.getFPGATimestamp();

            return Optional.of(myThingy);
        } catch (Exception e) {
            return Optional.empty();
        }
    }

    public String getName() {
        return "limelight";
    }
}
