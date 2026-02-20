package frc.robot.util;

import java.lang.annotation.Target;
import java.util.Optional;

import org.opencv.photo.Photo;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.sensors.ObjectDetectionCamera;

public class TargetUtils {

    /**
    * @param : robotPose and apriltags as int
    * @return : returns distance between robot and apriltag if apriltag is present
    */
    public static double getDistanceToFieldPos(Pose2d robotPose, int apriltag) {
        double distance = 0.0;
        Optional<Pose3d> tagPose = KnownLocations.getFieldLayout().getTagPose(apriltag);

        if (tagPose.isPresent()) {
            distance = robotPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
        }

        return distance;
    }


    // Following is based off https://www.chiefdelphi.com/t/is-there-a-builtin-function-to-find-the-angle-needed-to-get-one-pose2d-to-face-another-pose2d/455972
    /**
    * @param : robot pose, tagID
    * @return : Heading degrees required to face the April Tag
    */
    public static double getTargetHeadingToAprilTag(Pose2d robotPose, int tagId) {
        double heading = 0.0;
        Optional<Pose3d> tagPose = KnownLocations.getFieldLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
            Translation2d tagPoint = tagPose.get().getTranslation().toTranslation2d();
            Rotation2d targetRotation = tagPoint.minus(robotPose.getTranslation()).getAngle();
            heading = targetRotation.rotateBy(Rotation2d.fromDegrees(180.0)).getDegrees();
        }
        return heading;
    }
    /** 
    * @param : robotPose, point
    * @return : Target Rotation
    */
    public static Rotation2d getTargetHeadingToPoint(Pose2d robotPose, Translation2d point) {
        Rotation2d targetRotation = point.minus(robotPose.getTranslation()).getAngle();
        return  targetRotation;
    }

    public static double getDistanceToPoint(Pose2d pose, Translation2d point) {
        return pose.getTranslation().getDistance(point);
    }
     public static Rotation2d getTargetHeadingToFuel(Pose2d robotPose, PhotonTrackedTarget target) {
        Rotation2d targetRotation = Rotation2d.fromDegrees(-target.getYaw());
        return target.getYaw() != 0.0 ?
            robotPose.rotateBy(targetRotation).getRotation() :
            robotPose.getRotation();
    }

   public static Translation2d getFuelTranslation(PhotonTrackedTarget target, Pose2d robotPose, double distance) {
        return new Translation2d(distance + Units.inchesToMeters(5.0), getTargetHeadingToFuel(robotPose, target)).plus(robotPose.getTranslation());
   }

   
}


