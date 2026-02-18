// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.lang.annotation.Target;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.opencv.core.RotatedRect;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.path.PathPlannerPath;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SWERVE;
import frc.robot.sensors.AprilTagCamera;
import frc.robot.sensors.ProximitySensor;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.util.KnownLocations;
import frc.robot.util.PathUtils;
import frc.robot.util.SwerveUtils;
import frc.robot.util.TargetUtils;
import frc.robot.util.Shift;

public class Drivetrain extends SubsystemBase {

  private MAXSwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
  // private Pigeon2 pigeon;
  private Canandgyro gyro;
  private SwerveDrivePoseEstimator odometry;
  private SwerveDriveKinematics swerveKinematics;
  private AprilTagCamera leftCam, rightCam;
  private Field2d smartdashField;
  private PIDController rotationPIDController, xPIDController, yPIDController;
  private Pose2d startingPosition;
  private Shooter shooter;
  private Shift shift;


  private static final double ROTATION_P = 1.0 / 90.0, DIRECTION_P = 1 / 1.25, I = 0.0, D = 0.0;
  private final double THRESHOLD_DEGREES = 3.0;
  private final double THRESHOLD_SPEED = 0.5;
  private Translation2d robotVelocityVector;
  private Translation2d targetHubVector;
  private Translation2d targetHubVelocityVector;
  private Translation2d compensatedShotVector;

  private double headingToHub = 0.0;

  private double xDirStraight = 0.0, yDirStraight = 0.0;
  private Rotation2d rotationDirStraight = new Rotation2d();

  private Transform3d leftCamTransform3d = new Transform3d(
      new Translation3d(Units.inchesToMeters(9.0), Units.inchesToMeters(12.375), Units.inchesToMeters(9.0)),
      new Rotation3d(0.0, Rotation2d.fromDegrees(13).getRadians(), Rotation2d.fromDegrees(0).getRadians()));

  private Transform3d rightCamTransform3d = new Transform3d(
      new Translation3d(Units.inchesToMeters(9.0), Units.inchesToMeters(1.0), Units.inchesToMeters(9.0)),
      new Rotation3d(0.0, Rotation2d.fromDegrees(13).getRadians(), Rotation2d.fromDegrees(0).getRadians()));

  // // distance between wheels
  private final double WHEEL_WIDTH = 23.5; // distance between front/back wheels (in inches)
  private final double WHEEL_LENGTH = 28.5; // distance between left/right wheels (in inches)

  private Rotation2d gyroOffset = Rotation2d.fromDegrees(0); // Offset to apply to gyro for field oriented

  // Slew rate filter variables for controlling lateral acceleration
  private double currentAngularSpeed = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(SWERVE.MAGNITUDE_SLEW_RATE);
  private SlewRateLimiter angularSpeedLimiter = new SlewRateLimiter(SWERVE.ROTATIONAL_SLEW_RATE);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  /** Creates a new Drivetrain. */
  public Drivetrain(Shooter shooter) {
    // configure swerve modules
    /**
     * Configuring Driving and Turning motors for each Swerve Module
     */
    frontLeftModule = new MAXSwerveModule(CAN.DRIVING_FRONT_LEFT, CAN.TURNING_FRONT_LEFT, -Math.PI / 2);
    frontRightModule = new MAXSwerveModule(CAN.DRIVING_FRONT_RIGHT, CAN.TURNING_FRONT_RIGHT, 0);
    backLeftModule = new MAXSwerveModule(CAN.DRIVING_BACK_LEFT, CAN.TURNING_BACK_LEFT, Math.PI);
    backRightModule = new MAXSwerveModule(CAN.DRIVING_BACK_RIGHT, CAN.TURNING_BACK_RIGHT, Math.PI / 2);
    this.shooter = shooter;

    // leftSensors = new DistanceSensors(CAN.LEFT_INNER_LASER_CAN,
    // CAN.LEFT_OUTER_LASER_CAN, 430, 445, 400);
    // rightSensors = new DistanceSensors(CAN.RIGHT_INNER_LASER_CAN,
    // CAN.RIGHT_OUTER_LASER_CAN, 270, 220, 280);
    // backSensor = new DistanceSensors(CAN.BACK_LASER_CAN, 135.0); // check this
    // error


    gyro = new Canandgyro(CAN.GYRO_DRIVETRAIN);
    CanandgyroSettings gyroSettings = new CanandgyroSettings();
    gyro.setSettings(gyroSettings);
    // gyro.setPartyMode(1); //wooooo
    gyro.setPartyMode(0);

    rotationPIDController = new PIDController(ROTATION_P, I, D);
    rotationPIDController.enableContinuousInput(-180, 180);
    rotationPIDController.setTolerance(1.0);

    xPIDController = new PIDController(DIRECTION_P, I, D);
    xPIDController.setTolerance(0.01);

    yPIDController = new PIDController(DIRECTION_P, I, D);
    yPIDController.setTolerance(0.01);

    startingPosition = new Pose2d();

    // photonvision wrapper
    leftCam = new AprilTagCamera("LeftCamera", leftCamTransform3d);
    rightCam = new AprilTagCamera("RightCamera", rightCamTransform3d);

    smartdashField = new Field2d();
    SmartDashboard.putData("Swerve Odometry", smartdashField);

    /*
     * swerve modules relative to robot center --> kinematics object --> odometry
     * object
     */

    double widthFromCenter = Units.inchesToMeters(WHEEL_WIDTH) / 2;
    double lengthFromCenter = Units.inchesToMeters(WHEEL_LENGTH) / 2;

    swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(lengthFromCenter, widthFromCenter),
        new Translation2d(lengthFromCenter, -widthFromCenter),
        new Translation2d(-lengthFromCenter, widthFromCenter),
        new Translation2d(-lengthFromCenter, -widthFromCenter));

    // Initialize the robot odometry to the the field origin.
    // This will be updated by the selected Auton and DriveTrain.periodic()
    odometry = new SwerveDrivePoseEstimator(
        swerveKinematics,
        getRotation(),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        },
        new Pose2d(0, 0, getRotation()));

    shift = new Shift();
  }

  public PIDController getRotationalController() {
    return rotationPIDController;
  }

  public PIDController getXController() {
    return xPIDController;
  }

  public PIDController getYController() {
    return yPIDController;
  }

  public void setStartingPosition(Pose2d startPos) {
    this.startingPosition = startPos;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    backLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    backRightModule.resetEncoders();
  }

  /**
   * Resets the Gyro of robot, or facing to 0
   */
  public void resetGyro() {
    // pigeon.reset();
    gyro.setYaw(0);
    gyroOffset = Rotation2d.fromDegrees(0.0);
  }

  public void recalibrateGyro() {
    gyro.startCalibration();
  }

  /**
   * @param : Double as (xSpeed, ySpeed, angularSpeed)
   * @return : Assigning values to drive
   */
  public void drive(double xSpeed, double ySpeed, double angularSpeed) {
    drive(xSpeed, ySpeed, angularSpeed, true);
  }

  /**
   * @param : Double as (xSpeed, ySpeed, angularSpeed), fieldRelative
   * @return : Assigning Values to drive
   */
  public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative) {
    drive(xSpeed, ySpeed, angularSpeed, fieldRelative, false);
  }

  public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative, boolean rateLimit) {
    drive(xSpeed, ySpeed, angularSpeed, fieldRelative, rateLimit,
        () -> this.getPose().getRotation().plus(KnownLocations.getKnownLocations().zeroGyroRotation));
  }

  public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative, boolean rateLimit,
      Supplier<Rotation2d> rotationSupplier) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(SWERVE.DIRECTION_SLEW_RATE / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        } else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      } else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;

      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentAngularSpeed = angularSpeedLimiter.calculate(angularSpeed);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentAngularSpeed = angularSpeed;
    }

    double xSpeedDelivered = xSpeedCommanded * SWERVE.MAX_DIRECTION_SPEED;
    double ySpeedDelivered = ySpeedCommanded * SWERVE.MAX_DIRECTION_SPEED;
    double angularSpeedDelivered = currentAngularSpeed * SWERVE.MAX_ROTATIONAL_SPEED;

    ChassisSpeeds fieldRelativeSpeeds;

    if (fieldRelative) {
      fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
          angularSpeedDelivered, rotationSupplier.get());
    } else {
      fieldRelativeSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, angularSpeedDelivered);
    }

    drive(fieldRelativeSpeeds);
  }

  public void drive(ChassisSpeeds fieldRelativeSpeeds) {
    // general swerve speeds --> speed per module
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(fieldRelativeSpeeds);

    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SWERVE.MAX_DIRECTION_SPEED);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  public void lockSwerve() {
    // set wheels into X formation
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
    backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
    backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
  }

  /** update smartdash with trajectory */
  public void setTrajectorySmartdash(Trajectory trajectory, String type) {
    smartdashField.getObject(type).setTrajectory(trajectory);
  }

  public void setPoseSmartdash(Pose2d pose, String type) {
    smartdashField.getObject(type).setPose(pose);
  }

  /**
   * Set the odometry object to a predetermined pose
   * No need to reset gyro as it auto-applies offset
   * 
   * Used to set initial pose from an auton trajectory
   */
  public void resetPose(Pose2d pose) {
    // Set gyro offset for field orieneted rotatioon (zero faces away from alliance
    // station wall)
    gyroOffset = KnownLocations.getKnownLocations().zeroGyroRotation.plus(pose.getRotation());

    odometry.resetPosition(
        getRotation(),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        },
        pose);
  }

  public ChassisSpeeds getFieldRelativSpeeds() {
    return swerveKinematics.toChassisSpeeds(new SwerveModuleState[] {
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState()
    });
  }

  // meters/second
  // up is pos
  public double getXSpeeds() {
    return getFieldRelativSpeeds().vxMetersPerSecond;// NOT FIELD RELATIVE??????
  }

  // left is pos
  public double getYSpeeds() {
    return getFieldRelativSpeeds().vyMetersPerSecond;// has presumably the same issue as above
  }

  public Translation2d getVelocityVector() {
    return new Translation2d(getXSpeeds(), getYSpeeds())
        .rotateBy(getRotation().plus(KnownLocations.getKnownLocations().zeroGyroRotation));
  }

  /**
   * Get the robot relative rotation reported by the gyro (pigeon). Remember that
   * this should be CCW positive.
   * 
   * @return The rotation as read from the gyro.
   */
  public Rotation2d getRotation() {
    // Note: Unlike getAngle(), getRotation2d is CCW positive.
    return gyro.getRotation2d();
  }

  /**
   * Get the robot relative rotation reported by the gyro with an applied offset.
   * 
   * @param offset Offset to add to the gyro-supplied rotation.
   */
  public Rotation2d getRotation(Rotation2d offset) {
    return getRotation().plus(offset);
  }

  public Supplier<Double> getShootingAngleSupplier() {
    return () -> compensatedShotVector.getAngle().getDegrees();
  }

  public AprilTagCamera getAprilTagCamera() {
    return this.leftCam;
  }

  public boolean isTargetPresent() {
    Optional<EstimatedRobotPose> result = leftCam.getGlobalPose();
    return result.isPresent();
  }

  public boolean isNotMoving() {
    return Math.abs(getXSpeeds()) < THRESHOLD_SPEED && Math.abs(getYSpeeds()) < THRESHOLD_SPEED;
  }

  public boolean isAtPose(Pose2d target) {
    return isAtPose(target, getXController().getErrorTolerance(), getYController().getErrorTolerance());
  }

  public boolean isAtPose(Pose2d target, double xTolerance, double yTolerance) {
    return Math.abs(target.getX() - getPose().getX()) < xTolerance
        && Math.abs(target.getY() - getPose().getY()) < yTolerance;
  }

  public boolean isAtPose(Pose2d target, double tolerance) {
    return isAtPose(target, tolerance, tolerance);
  }

  public double getMinimumAbiguity(List<PhotonTrackedTarget> targetsUsed) {
    double min = 1.0;

    for (PhotonTrackedTarget photonTrackedTarget : targetsUsed) {
      min = Math.min(min, photonTrackedTarget.getPoseAmbiguity());
    }

    return min;
  }

  public double getSafeBumpingAngle() {
    double currentHeading = getPose().getRotation().getDegrees();
    if (currentHeading >= 0.0 && currentHeading <= 90.0) {
      return 45.0;
    } else if (currentHeading >= 90.0 && currentHeading <= 180.0) {
      return 135.0;
    } else if (currentHeading <= -90.0 && currentHeading >= -180.0) {
      return -135.0;
    } else if (currentHeading <= 0.0 && currentHeading >= -90.0) {
      return -45.0;
    } else {
      return 0.0;
    }
  }

  public double getXSpeedCappedStraightDrive() {
    return xDirStraight;
  }

  public double getYSpeedCappedStraightDrive() {
    return yDirStraight;
  }

  public Rotation2d getRotationBeforeStraightDrive() {
    return rotationDirStraight;
  }

  public void setXDirStraight(double speed) {
    // Translation2d current = new Translation2d(getXSpeeds(), getYSpeeds());
    // double magnitude = current.getNorm();
    // if (magnitude > 1e-6) {
    // Translation2d scaled = current.div(magnitude).times(.1);
    // xDirStraight = scaled.getX();
    // } else {
    // xDirStraight = 0.0;
    // }
    double xSpeeds = getVelocityVector().getX();
    if (Math.abs(xSpeeds) > 1e-6) {
      xDirStraight = xSpeeds;
    } else {
      xDirStraight = 0.0;
    }
  }

  public void setYDirStraight(double speed) {
    // Translation2d current = new Translation2d(getXSpeeds(), getYSpeeds());
    // double magnitude = current.getNorm();
    // if (magnitude > 1e-6) {
    // Translation2d scaled = current.div(magnitude).times(.1);
    // yDirStraight = scaled.getY();
    // } else {
    // yDirStraight = 0.0;
    // }
    double ySpeeds = getVelocityVector().getY();
    if (Math.abs(ySpeeds) > 1e-6) {
      yDirStraight = ySpeeds;
    } else {
      yDirStraight = 0.0;
    }
  }

  public void setRotationBeforeStraightDrive(Rotation2d rotation) {
    rotationDirStraight = rotation;
  }

  public double getHeadingToHub() {
    return headingToHub;
  }

  public Shift getShift() {
    return shift;
  }

  public Translation2d getCompensatedVector() {
    return compensatedShotVector;
  }

  public Zone getCurrentZone() {
    KnownLocations locs = KnownLocations.getKnownLocations();
    double x = getPose().getX();
    double y = getPose().getY();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (x < locs.NEUTRAL_EDGE.getX() && x > locs.STARTING_LINE.getX()) {
        return Zone.BETWEEN;
      } else if (x < locs.STARTING_LINE.getX() && y > locs.HUB.getY()) {
        return Zone.ALLIANCE_LEFT;
      } else if (x < locs.STARTING_LINE.getX() && y < locs.HUB.getY()) {
        return Zone.ALLIANCE_RIGHT;
      } else if (x > locs.NEUTRAL_EDGE.getX() && y > locs.HUB.getY() + Units.inchesToMeters(23.5)) {
        return Zone.NEUTRAL_LEFT;
      } else if (x > locs.NEUTRAL_EDGE.getX() && y < locs.HUB.getY() - Units.inchesToMeters(23.5)) {
        return Zone.NEUTRAL_RIGHT;
      } else if (x > locs.NEUTRAL_EDGE.getX() && y < locs.HUB.getY() + Units.inchesToMeters(23.5)
          && y > locs.HUB.getY() - Units.inchesToMeters(23.5)) {
        return Zone.NEUTRAL_MIDDLE;
      }
    } else {
      if (x > locs.NEUTRAL_EDGE.getX() && x < locs.STARTING_LINE.getX()) {
        return Zone.BETWEEN;
      } else if (x > locs.STARTING_LINE.getX() && y < locs.HUB.getY()) {
        return Zone.ALLIANCE_LEFT;
      } else if (x > locs.STARTING_LINE.getX() && y > locs.HUB.getY()) {
        return Zone.ALLIANCE_RIGHT;
      } else if (x < locs.NEUTRAL_EDGE.getX() && y < locs.HUB.getY() - Units.inchesToMeters(23.5)) {
        return Zone.NEUTRAL_LEFT;
      } else if (x < locs.NEUTRAL_EDGE.getX() && y > locs.HUB.getY() + Units.inchesToMeters(23.5)) {
        return Zone.NEUTRAL_RIGHT;
      } else if (x < locs.NEUTRAL_EDGE.getX() && y > locs.HUB.getY() - Units.inchesToMeters(23.5)
          && y < locs.HUB.getY() + Units.inchesToMeters(23.5)) {
        return Zone.NEUTRAL_MIDDLE;
      }
    }
    return Zone.BETWEEN; // should never get here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shift.updateStatesForTeleop();

    headingToHub = TargetUtils
        .getTargetHeadingToPoint(getPose(), KnownLocations.getKnownLocations().HUB.getTranslation()).getDegrees();

    robotVelocityVector = new Translation2d(getXSpeeds(), getYSpeeds());
    
    // TODO: might want to use current velocity of the shooter instead of theoretical
    Translation2d exitVelocityVector = new Translation2d(shooter.getShootingRPM(),
        Rotation2d.fromDegrees(headingToHub));
    compensatedShotVector = exitVelocityVector.minus(robotVelocityVector);

    odometry.update(
        getRotation(),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        });

    Optional<EstimatedRobotPose> leftResult = leftCam.getGlobalPose();
    if (leftResult.isPresent() && !leftCam.rejectUpdate(leftResult.get())) {
      // Uncomment the following to check camera position on robot
      // Pose3d estimatedPose = result.get().estimatedPose;
      // SmartDashboard.putNumber("Cam/Yaw", estimatedPose.getRotation().getZ());
      // SmartDashboard.putNumber("Cam/Pitch", estimatedPose.getRotation().getY());
      // SmartDashboard.putNumber("Cam/Roll", estimatedPose.getRotation().getX());
      odometry.addVisionMeasurement(leftResult.get().estimatedPose.toPose2d(), leftResult.get().timestampSeconds);
    }

    Optional<EstimatedRobotPose> rightResult = rightCam.getGlobalPose();
    if (rightResult.isPresent() && !rightCam.rejectUpdate(rightResult.get())) {
      // Uncomment the following to check camera position on robot
      // Pose3d estimatedPose = result.get().estimatedPose;
      // SmartDashboard.putNumber("Cam/Yaw", estimatedPose.getRotation().getZ());
      // SmartDashboard.putNumber("Cam/Pitch", estimatedPose.getRotation().getY());
      // SmartDashboard.putNumber("Cam/Roll", estimatedPose.getRotation().getX());
      odometry.addVisionMeasurement(rightResult.get().estimatedPose.toPose2d(), rightResult.get().timestampSeconds);
    }

    smartdashField.setRobotPose(getPose());

    SmartDashboard.putNumber("Drivetrain/CurrentPose X", getPose().getX());
    SmartDashboard.putNumber("Drivetrain/CurrentPose Y", getPose().getY());
    SmartDashboard.putNumber("Drivetrain/getRotation", getRotation().getDegrees());
    SmartDashboard.putBoolean("Drivetrain/isNotMoving", isNotMoving());
    SmartDashboard.putNumber("Drivetrain/CurrentPose Rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Drivetrain/Drive Angle", getRotation().getDegrees());
    SmartDashboard.putNumber("Drivetrain/gyroOffset", gyroOffset.getDegrees());
    SmartDashboard.putBoolean("Drivetrain/isLeftCamUpdating",
        leftResult.isPresent() && !leftCam.rejectUpdate(leftResult.get()));
    SmartDashboard.putBoolean("Drivetrain/isRightCamUpdating",
        rightResult.isPresent() && !rightCam.rejectUpdate(rightResult.get()));
    SmartDashboard.putNumber("Drivetrain/safeBumpingAngle", getSafeBumpingAngle());
    SmartDashboard.putNumber("Drivetrain/xVelocity", getVelocityVector().getX());
    SmartDashboard.putNumber("Drivetrain/yVelocity", getVelocityVector().getY());
    SmartDashboard.putNumber("Drivetrain/xSpeedCappedStraight", getXSpeedCappedStraightDrive());
    SmartDashboard.putNumber("Drivetrain/ySpeedCappedStraight", getYSpeedCappedStraightDrive());
    SmartDashboard.putData(smartdashField);
    SmartDashboard.putNumber("Drivetrain/shootHereAngle", compensatedShotVector.getAngle().getDegrees());
    SmartDashboard.putNumber("Drivetrain/headingToHub", headingToHub);
    SmartDashboard.putNumber("Drivetrain/headingThingIDK",
        this.getPose().getRotation().plus(KnownLocations.getKnownLocations().zeroGyroRotation).getDegrees());
    SmartDashboard.putString("Drivetrain/currentZone", getCurrentZone().getString());
    SmartDashboard.putBoolean("Shift/isOurHubActive", shift.isOurHubActive());
    SmartDashboard.putString("Drivetrain/gameMessage", DriverStation.getGameSpecificMessage());
    SmartDashboard.putNumber("Drivetrain/gyroReading", gyro.getRotation2d().getDegrees());
    SmartDashboard.putBoolean("Drivetrain/gyroIsCalibrating", gyro.isCalibrating());
  }

  public enum Zone {
    ALLIANCE_LEFT("allianceLeft"),
    ALLIANCE_RIGHT("allianceRight"), // do we need left and right?
    NEUTRAL_LEFT("neutralLeft"),
    NEUTRAL_MIDDLE("neutralMiddle"),
    NEUTRAL_RIGHT("neutralRight"),
    BETWEEN("between");

    public String zone;

    Zone(String zone) {
      this.zone = zone;
    }

    public String getString() {
      return zone;
    }
  }
}