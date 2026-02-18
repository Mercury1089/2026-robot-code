// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.PathUtils;
import frc.robot.util.TargetUtils;


/** Wrapper for PhotonCamera class */
public class ObjectDetectionCamera extends PhotonCamera {

    // Camera angles for calculating target distance.
    // Used for https://javadocs.photonvision.org/org/photonvision/PhotonUtils.html#calculateDistanceToTargetMeters(double,double,double,double)
    private static final String DEFAULT_CAM_NAME = "ObjectDetectionCam";
    private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(18.0); // height on robot (meters) TODO: CHECK THIS
    private final double CAMERA_PITCH_RADIANS = Rotation2d.fromDegrees(-25.0).getRadians(); // tilt of our camera (radians)
    private final double TARGET_HEIGHT_METERS = 0.0; // may need to change 
    private final double MID_SCREEN = 400; //HALF OF RESOLUTION WIDTH IN PIXES

    public ObjectDetectionCamera() {
        super(DEFAULT_CAM_NAME);
    }

    public double getYaw() {
        /* The yaw of the target in degrees (positive right). */
        var result = getLatestResult();
        return result.hasTargets() ? 
            result.getBestTarget().getYaw() :
            0.0;
    }

    public double getPitch() {
        /* The pitch of the target in degrees (positive up). */
        var result = getLatestResult();
        return result.hasTargets() ? 
            result.getBestTarget().getPitch() :
            0.0;
    }

    public double getSkew() {
        /* The skew of the target in degrees (counter-clockwise positive). */
        var result = getLatestResult();
        return result.hasTargets() ? 
            result.getBestTarget().getSkew() :
            0.0;
    }

    public int getTargetCount() {
        var result = getLatestResult();

        return result.hasTargets() ? 
            result.getTargets().size() :
            0;
    }

    public double getDistanceToTarget() {
        PhotonPipelineResult result = getLatestResult();
        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
                Units.degreesToRadians(result.getBestTarget().getPitch())
            );
            return range;
        }
        return 0.0;
    }

    public Pose2d getCoordinateOfHighestConcentration(Drivetrain drivetrain) {
        var result = getLatestResult();

        if (!result.hasTargets()) return new Pose2d(); // dont do this fix this

        List<PhotonTrackedTarget> targets = result.targets;
        List<Double[]> fuelPosesLeft = new ArrayList<>();
        List<Double[]> fuelPosesRight = new ArrayList<>();
        int countOfFuelOnLeft = 0;
        int countOfFuelOnRight = 0;
        for (int i = 0; i < targets.size(); i++) {

            double xCoordinate = targets.get(i).getDetectedCorners().get(0).x;
            double yCoordinate = targets.get(i).getDetectedCorners().get(0).x;


            if (xCoordinate < 400) {
                countOfFuelOnLeft++;
                fuelPosesLeft.add(new Double[]{xCoordinate, yCoordinate});
            } else {
                countOfFuelOnRight++;
                fuelPosesRight.add(new Double[]{xCoordinate, yCoordinate});
            }
        }

        return new Pose2d();

        //get avereage point next we have all the coordinates
    }
    

    // public Transform3d transformToNote() {
    //     Transform3d pose = getLatestResult().getBestTarget().getBestCameraToTarget();
    //     return pose;
    // }

    
}