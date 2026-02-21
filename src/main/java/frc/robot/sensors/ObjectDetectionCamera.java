// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleBinaryOperator;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.MercMath;
import frc.robot.util.TargetUtils;


/** Wrapper for PhotonCamera class */
public class ObjectDetectionCamera extends PhotonCamera {

    // Camera angles for calculating target distance.
    // Used for https://javadocs.photonvision.org/org/photonvision/PhotonUtils.html#calculateDistanceToTargetMeters(double,double,double,double)
    private static final String DEFAULT_CAM_NAME = "FuelCam";
    private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20.5); // height on robot (meters) TODO: CHECK THIS
    private final double CAMERA_PITCH_RADIANS = Rotation2d.fromDegrees(-15.0).getRadians(); // tilt of our camera (radians)
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

    public double getDistanceToTarget(PhotonTrackedTarget target) {
       double range = PhotonUtils.calculateDistanceToTargetMeters(
               CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS,
               Units.degreesToRadians(target.getPitch()));
       return range;
   }

    //relative to robot; x is forwards. i think
    public Translation2d getTranslationOfHighestConcentration(Drivetrain drivetrain) {
        var result = getLatestResult();

        if (!result.hasTargets()) return new Translation2d(); 

        List<PhotonTrackedTarget> fuelTargets = result.targets;
        // List<PhotonTrackedTarget> fuelPosesLeft = new ArrayList<>();
        // List<PhotonTrackedTarget> fuelPosesRight = new ArrayList<>();

        List<PhotonTrackedTarget> filtered = new ArrayList<PhotonTrackedTarget>();

        for (int i = 0; i < fuelTargets.size(); i++) {

            double x = fuelTargets.get(i).getMinAreaRectCorners().get(0).x;
            System.out.println(x);
            // sort each fuel int the right or left half
            if (x > 200 && x < 600) {
                filtered.add(fuelTargets.get(i));
            }
        }

        // int leftCount = fuelPosesLeft.size();
        // int rightCount = fuelPosesRight.size();

        // if (leftCount == 0 && rightCount == 0) {
        //     return new Translation2d();
        // }

        

        // // List<PhotonTrackedTarget> mostCountOfFuel = leftCount > rightCount ? fuelPosesLeft : fuelPosesRight;
        // List<PhotonTrackedTarget> mostCountOfFuel = result.targets;
        // List<PhotonTrackedTarget> filtered = new ArrayList<PhotonTrackedTarget>();
        // List<Double> allXDoubles = new ArrayList<Double>();
        // for (PhotonTrackedTarget photonTrackedTarget : filtered) {
        //     allXDoubles.add(photonTrackedTarget.getMinAreaRectCorners().get(0).x);
        // }
        // allXDoubles = MercMath.removeOutliersIQR(allXDoubles);

        // for (int i = 0; i < fuelTargets.size(); i++) {
        //     if(!allXDoubles.contains(fuelTargets.get(i).getMinAreaRectCorners().get(0).x)) {
        //         fuelTargets.remove(i);
        //     }
        // }

        double totalX = 0.0;
        double totalY = 0.0;

        for (PhotonTrackedTarget photonTrackedTarget : filtered) {
            Translation2d fuelTranslation2d = TargetUtils.getFuelTranslation(photonTrackedTarget, drivetrain.getPose(), getDistanceToTarget(photonTrackedTarget));
            totalX += fuelTranslation2d.getX();
            totalY += fuelTranslation2d.getY();
        }

        return new Translation2d(totalX / fuelTargets.size(), totalY / fuelTargets.size());
    }

    // public Transform3d transformToNote() {
    //     Transform3d pose = getLatestResult().getBestTarget().getBestCameraToTarget();
    //     return pose;
    // }

    
}