package frc.robot.subsystems.outtake;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.FeedbackSensor;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.Zone;
import frc.robot.util.KnownLocations;
import frc.robot.util.MercMath;
import frc.robot.util.TargetUtils;

/** Shooter subsystem with two NEO motors controlled by SPARK MAX closed-loop velocity control. */
public class Shooter extends SubsystemBase {
    private final SparkFlex leader;
    private final SparkFlex follower_first_right, follower_second_left, follower_third_left;

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController leaderClosedLoop;
    private LaserCan lc;
    private Drivetrain drivetrain;
    private double THRESHOLD_RPM = 100.0; // TODO: tune this threshold
    private final double MAX_VOLTAGE = 10.5;

    private double setRPM = 0.0, desiredFireRPM = 0.0;
    private double smartdashRPM = 0.0;

    double kP = 0.000025, kS = 0.2, freeRPMs = 5400.0;

    public Shooter(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        leader = new SparkFlex(Constants.CAN.SHOOTER, MotorType.kBrushless); // all the way to the right
        follower_first_right = new SparkFlex(Constants.CAN.SHOOTER_FOLLOWER_FIRST, MotorType.kBrushless); // one in
        follower_second_left = new SparkFlex(Constants.CAN.SHOOTER_FOLLOWER_SECOND, MotorType.kBrushless);
        follower_third_left = new SparkFlex(Constants.CAN.SHOOTER_FOLLOWER_THIRD, MotorType.kBrushless);

        lc = new LaserCan(Constants.CAN.LASER_CAN_SHOOTER);

        // Build the shooter SparkMaxConfig inline (previously in ShooterConfigs)
        SparkFlexConfig leader_config = new SparkFlexConfig();
        leader_config.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
        // use RPM units for encoder velocity here (1.0 = pass-through)
        leader_config.encoder.velocityConversionFactor(1.0);

        double kI = 0.0;
        double kD = 0.0;
        // double nominalVoltage = 12.0;
        // double velocityFF = MAX_VOLTAGE / Constants.NEO_MOTOR_CONSTANTS.VORTEX_FREE_SPEED_RPMS;
        double velocityFF = (MAX_VOLTAGE-kS) / freeRPMs;

        leader_config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .outputRange(-1.0, 1.0)
            .feedForward.kV(velocityFF).kS(kS);

        leader_config.inverted(false)
            .voltageCompensation(MAX_VOLTAGE);

        // Build the shooter SparkMaxConfig inline (previously in ShooterConfigs)
        SparkFlexConfig follower_config_left = new SparkFlexConfig();
        follower_config_left.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(MAX_VOLTAGE);
        // use RPM units for encoder velocity here (1.0 = pass-through)
        follower_config_left.encoder.velocityConversionFactor(1.0);
        follower_config_left.follow(Constants.CAN.SHOOTER, true);//assumes invert of leader affects followers. which is true

        SparkFlexConfig follower_config_right = new SparkFlexConfig();
        follower_config_right.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(MAX_VOLTAGE);
        follower_config_right.encoder.velocityConversionFactor(1.0);
        follower_config_right.follow(Constants.CAN.SHOOTER, false);


        // Apply configuration and persist parameters (consistent with existing project style)
        leader.configure(leader_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower_first_right.configure(follower_config_right, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower_second_left.configure(follower_config_left, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower_third_left.configure(follower_config_left, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = leader.getEncoder();
        leaderClosedLoop = leader.getClosedLoopController();

        SmartDashboard.putNumber("Shooter/kP", kP);
        SmartDashboard.putNumber("Shooter/freeRpms", 5400.0);
        SmartDashboard.putNumber("Shooter/kS", kS);
        SmartDashboard.putNumber("Shooter/smartDashRPM", 0.0);
    }

    /**
     * Set shooter velocity in RPM.
     * @param rpm target velocity in rotations per minute
     */
    public void setVelocityRPM(double rpm) {
        setRPM = rpm;
        leaderClosedLoop.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void setSpeed(double speed) {
        leader.set(speed);
    }

    /** Stop the shooter (velocity -> 0 RPM). avoid calling this*/
    public void stop() {
        setVelocityRPM(0.0);
    }

    /**
     * @return current encoder velocity in RPM (matching ShooterConfigs velocityConversionFactor)
     */
    public double getVelocityRPM() {
        return encoder.getVelocity();
    }

    public void goToSetRPM() {
        setVelocityRPM(setRPM);
    }

    public void increaseRPM() {
        setRPM = setRPM + 50.0;
    }

    public void decreaseRPM() {
        setRPM = setRPM - 50.0;
    }

    public double getSetRPM() {
        return setRPM;
    }
    
    // Make sure to return RPM, as in the Drivetrain periodic we convert this to m/s using MercMath.RPMToMetersPerSecond()
    // Write an if-statement to see if you are passing or shooting, and return the appropriate RPM for each case
    public double getStaticShootingRPM(boolean passing) {
        Translation2d point = new Translation2d();

        if(drivetrain.isDrivetrainInAllianceZone() || drivetrain.getCurrentZone() == Zone.BETWEEN) {
            point = KnownLocations.getKnownLocations().HUB.getTranslation();
        } else if(drivetrain.getCurrentZone() == Zone.NEUTRAL_LEFT) {
            point = KnownLocations.getKnownLocations().PASSING_TARGET_LEFT.getTranslation();
        } else if(drivetrain.getCurrentZone() == Zone.NEUTRAL_RIGHT) {
            point = KnownLocations.getKnownLocations().PASSING_TARGET_RIGHT.getTranslation();
        }

        double d = TargetUtils.getDistanceToPoint(drivetrain.getPose(), point);

        if(drivetrain.isDrivetrainInAllianceZone()) {
            return 1152 + (873 * d) + (-309 * Math.pow(d, 2)) + (65.4 * Math.pow(d, 3)) + (-5.11 * Math.pow(d, 4));
        } else {
            if (!passing) {
                return 1000.0;
            }
            double final_rpm = (159 * d) + 1553;
            return Math.min(3500.0, final_rpm); // Enter the passing function (this is the only spot to enter any passing information)
        }
    }

    // public void setFireVelocity() {
    //     setVelocityRPM(getStaticShootingRPM());
    //     // setVelocityRPM(MercMath.metersPerSecondToRPM(drivetrain.getCompensatedVector().getNorm(), 2.0));
    // }

    public boolean isShooterAtManualShotRPM() {
        return Math.abs(getVelocityRPM()) > 2500;
    }

    public boolean isAtShootingRPM() {
        return Math.abs(getSetRPM() - getVelocityRPM()) < THRESHOLD_RPM;
    }
    public boolean fuelInShooter(){
        LaserCan.Measurement measurement;
        measurement = lc.getMeasurement();
        if (measurement != null){
            return  (measurement.distance_mm < 450); //TODO: tweak values as necessary
        }
        else{
            return false;
        }
    }

    public double getSmartDashRPM() {
        return smartdashRPM;
    }

    public void periodic() {
        SmartDashboard.putNumber("Shooter/RPM", getVelocityRPM());
        SmartDashboard.putNumber("Shooter/setRPM", setRPM);

        SmartDashboard.putBoolean("Shooter/isAtRPM", isAtShootingRPM());
        // // SmartDashboard.putBoolean("Shooter/isFuelInShooter", fuelInShooter());

        // kP = SmartDashboard.getNumber("Shooter/kP", kP);
        // kS = SmartDashboard.getNumber("Shooter/kS", kS);
        // double ff = (MAX_VOLTAGE-kS) / SmartDashboard.getNumber("Shooter/freeRpms", freeRPMs);
        // smartdashRPM = SmartDashboard.getNumber("Shooter/smartDashRPM", 0.0);

        // if (DriverStation.isDisabled()) {
        //     setRPM = smartdashRPM;
        //     SparkFlexConfig leader_config = new SparkFlexConfig();

        //     double kI = 0.0;
        //     double kD = 0.0;
        //     // double nominalVoltage = 12.0;
        //     leader_config.closedLoop
        //             .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //             .pid(kP, kI, kD)
        //             .outputRange(-1.0, 1.0).feedForward.kV(ff).kS(kS);
            
        //     leader.configure(leader_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // } 
        
        // SmartDashboard.putNumber("Shooter/currentkS", leader.configAccessor.closedLoop.feedForward.getkS());
    }
}
