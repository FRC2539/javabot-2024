package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

public class AprilTagIOPinhole implements AprilTagIO {
    private CameraInfoIO cameraInfoIO;
    private double cameraAngleVerticle;
    private double cameraHeight;
    private double aprilTagHeight;
    private double cameraAngleHorizontal;
    private SwerveDriveSubsystem swerveDrive;
    
    public AprilTagIOPinhole(CameraInfoIO cameraInfo, double cameraAngleVerticle, double cameraHeight, double aprilTagHeight, double cameraAngleHorizontal, SwerveDriveSubsystem boop) {
        this.cameraInfoIO = cameraInfo;
        
        // how many radians back is your limelight rotated from perfectly vertical?
        this.cameraAngleVerticle = cameraAngleVerticle;

        // distance from the center of the Limelight lens to the floor
        this.cameraHeight = cameraHeight;

        // distance from the target to the flooraprilTagHeight; 
        this.aprilTagHeight = aprilTagHeight;

        // radians additional to get the camera angle from the gyro angle
        this.cameraAngleHorizontal = cameraAngleHorizontal;

        this.swerveDrive = boop;
    }

    public Optional<AprilTagIOInputs> updateInputs() {
        return cameraInfoIO.updateInputs().map(
            inputs -> {
                AprilTagIOInputs myInputs = new AprilTagIOInputs();

                double angleToGoalRadians = cameraAngleVerticle + inputs.ty;

                //calculate distance
                double distance = (aprilTagHeight - cameraHeight) / Math.tan(angleToGoalRadians);

                AprilTag aprilTag = Constants.FieldConstants.aprilTags.get(inputs.targetID);

                Translation2d translationRobotToApriltag = new Translation2d(distance, new Rotation2d(cameraAngleHorizontal).plus(swerveDrive.getGyroRotation()));

                Translation2d nextPoseEstimate = aprilTag.pose.toPose2d().getTranslation().plus(translationRobotToApriltag.unaryMinus());

                var poseEstimate = new Pose2d(nextPoseEstimate, swerveDrive.getGyroRotation());

                myInputs.poseEstimate = poseEstimate;
                myInputs.poseEstimate3d = new Pose3d(poseEstimate);
                myInputs.targetDistance = distance;
                myInputs.timestamp = inputs.timestamp;

                return myInputs;
            }
        );
    }
}
