// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.lang.annotation.Target;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.KnownLocations;
import frc.robot.util.TargetUtils;

/** Wrapper for PhotonCamera class */
public class AprilTagCamera extends PhotonCamera {

    // AprilTagCamera 3d Pose on robot
    // Uses coordinates described here: https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html#camera-coordinate-frame
    // private static final String DEFAULT_CAM_NAME = "AprilTagCamera";
    // private static final double DEFAULT_CAM_X = Units.inchesToMeters(13.5); // 14.75in behind center
    // private static final double DEFAULT_CAM_Y = 0.0; // centered in robot Y
    // private static final double DEFAULT_CAM_Z = Units.inchesToMeters(6.5); // 21.25in up from center

    // private static final double DEFAULT_CAM_ROTATION = Rotation2d.fromDegrees(0).getRadians(); // rotation relative to robot front (radians)
    //private static final double DEFAULT_CAM_TILT = Rotation2d.fromDegrees(20).getRadians(); // tilt relative to floor (raians)

    // private static final double TARGET_HEIGHT = 0.36; // may need to change - DO WE NEED THIS?
    // private static final double CAMERA_HEIGHT = DEFAULT_CAM_Z; // height on robot (meters) - DO WE NEED THIS?


    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator estimator;

    private Matrix<N3, N1> curStdDevs;
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public AprilTagCamera(String name, Transform3d robotToCam) {
        super(name);
        fieldLayout = KnownLocations.getFieldLayout();
        // Transform3d robotToCam = new Transform3d(
        //     new Translation3d(DEFAULT_CAM_X, DEFAULT_CAM_Y, DEFAULT_CAM_Z), new Rotation3d(0.0, DEFAULT_CAM_TILT, DEFAULT_CAM_ROTATION)
        // );
        // Uncomment the following to silence missing camera errors
        // PhotonCamera.setVersionCheckEnabled(false);
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    }

    public Optional<EstimatedRobotPose> getGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : getAllUnreadResults()) {
            visionEst = estimator.update(change);
            // TODO: See example: https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }

    public double getDistanceToClosestTag(EstimatedRobotPose pose) {
        Pose2d camPose2d = pose.estimatedPose.toPose2d();
        int aprilTag;

        if (pose.targetsUsed.size() > 0) {
            aprilTag = pose.targetsUsed.get(0).getFiducialId();
        } else {
            return Double.MAX_VALUE;
        }

        return TargetUtils.getDistanceToFieldPos(camPose2d, aprilTag);
    }
    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public boolean rejectUpdate(EstimatedRobotPose estimatedPose) {
        List<PhotonTrackedTarget> usedTags = estimatedPose.targetsUsed;

        double largestArea = 0.0;

        for (PhotonTrackedTarget t : usedTags) {
            if (t.getArea() > largestArea) {
                largestArea = t.getArea();
            }
        }

        if (largestArea < 0.15) {
            return true;
        }
        
        if (usedTags.size() == 1 && usedTags.get(0).getPoseAmbiguity() > 0.2) {
            return true;
        }

        return false;
    }

    public double getYaw() {
        /* The yaw of the target in degrees (positive right). */
        return getLatestResult().getBestTarget().getYaw();
    }

    public double getPitch() {
        /* The pitch of the target in degrees (positive up). */
        return getLatestResult().getBestTarget().getPitch();
    }

    public double getSkew() {
        /* The skew of the target in degrees (counter-clockwise positive). */
        return getLatestResult().getBestTarget().getSkew();
    }

    public double getApriltagID() {
        return getLatestResult().getBestTarget().getFiducialId();
    }
}

