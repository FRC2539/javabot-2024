package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

public class ExampleLimelight {

    final String LIMELIGHT_NAME = "limelight";

    public void getData() {

        final String LIMELIGHT_NAME = "limelight";

        // Basic targeting data
        // Get horizontal (tx) and vertical (ty) offset from crosshair to target in degrees
        double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        double ty = LimelightHelpers.getTY(LIMELIGHT_NAME);

        // Target area in percent (0 for none of the image and 100 for all of it)
        double ta = LimelightHelpers.getTA(LIMELIGHT_NAME);

        // Do you have a valid target?
        boolean hasTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);

        // Horizontal offset from principal pixel/point to target in degrees
        double txnc = LimelightHelpers.getTXNC(LIMELIGHT_NAME);  
        double tync = LimelightHelpers.getTYNC(LIMELIGHT_NAME);

        // NOTE: Positive x is to the right, positive y is up.

        // Switch pipelines on the Limelight through code
        LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);

        // Set a custom crop window for improved performance (-1 to 1 for each value)
        LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

        tx = tx;
        ty = ty;
        ta = ta;
        hasTarget = hasTarget;
        txnc = txnc;
        tync = tync;
    }

    public void periodic() {
        if (LimelightHelpers.getTV(LIMELIGHT_NAME)) {
            setTurning(0);
            return;
        }

        // Gets the X position (left is -1, right is +1)
        double currentTargetX = LimelightHelpers.getTX(LIMELIGHT_NAME);

        // If the robot is looking at the target
        if (currentTargetX < 2 && currentTargetX > -2) {
            setTurning(0);
            return;
        }

        // If the robot is looking to the...
        if (currentTargetX > 0) {
            // left of the target it will turn right
            setTurning(-1);
        } else {
            // right of the target it will turn left
            setTurning(1);
        }
    }

    public void setTurning(double speed) {}

    public void trackAprilTag() {
        // Set the priority tag that your camera should track
        LimelightHelpers.setPriorityTagID(LIMELIGHT_NAME, 7);

        // Set the list of tags that your camera should track
        // (It ignores the others)
        LimelightHelpers.SetFiducialIDFiltersOverride(LIMELIGHT_NAME, new int[]{7,8,3,4});
    }

    public void updateVision() {
        LimelightHelpers.SetRobotOrientation(
            LIMELIGHT_NAME, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
             0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        if(Math.abs(getGyroRate()) > 720) return;

        // if there is no tag, ignore vision updates
        if(mt2.tagCount == 0) return;

        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
    }

    public void updateVisionMegaTag1() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
      
        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7) return;

            if(mt1.rawFiducials[0].distToCamera > 3) return;
        }

        if(mt1.tagCount == 0) return;

        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
    }

    public Optional<Pose2d> getPoseFromLimelight() {
        // update MegaTag2 using gyro orientation
        LimelightHelpers.SetRobotOrientation(
            LIMELIGHT_NAME, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
             0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        if(Math.abs(getGyroRate()) > 720) return Optional.empty();

        // if there is no tag, ignore vision updates
        if(mt2.tagCount == 0) return Optional.empty();
        

        return Optional.of(mt2.pose);
    }

    public double getGyroRate() {
        return 0;
    }

    private SwerveDrivePoseEstimator m_poseEstimator;
}
