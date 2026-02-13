package frc.robot.subsystems.outtake;

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

/** Shooter subsystem with two NEO motors controlled by SPARK MAX closed-loop velocity control. */
public class Shooter extends SubsystemBase {
    private final SparkFlex leader;
    private final SparkFlex follower;

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController leaderClosedLoop;
    private final SparkClosedLoopController followerClosedLoop;
    private LaserCan lc;

    public Shooter() {
        leader = new SparkFlex(Constants.CAN.SHOOTER, MotorType.kBrushless);
        follower = new SparkFlex(Constants.CAN.SHOOTER_FOLLOWER, MotorType.kBrushless);

        // Build the shooter SparkMaxConfig inline (previously in ShooterConfigs)
        SparkFlexConfig leader_config = new SparkFlexConfig();
        leader_config.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
        // use RPM units for encoder velocity here (1.0 = pass-through)
        leader_config.encoder.velocityConversionFactor(1.0);

        double kP = 0.001;
        double kI = 0.0;
        double kD = 0.0;
        double nominalVoltage = 12.0;
        double velocityFF = nominalVoltage / Constants.NEO_MOTOR_CONSTANTS.VORTEX_FREE_SPEED_RPMS;

        leader_config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .outputRange(-1.0, 1.0)
            .feedForward.kV(velocityFF);

        leader_config.inverted(true);

        // Build the shooter SparkMaxConfig inline (previously in ShooterConfigs)
        SparkFlexConfig follower_config = new SparkFlexConfig();
        follower_config.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
        // use RPM units for encoder velocity here (1.0 = pass-through)
        follower_config.encoder.velocityConversionFactor(1.0);
        follower_config.follow(Constants.CAN.SHOOTER, true);

        // Apply configuration and persist parameters (consistent with existing project style)
        leader.configure(leader_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(follower_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = leader.getEncoder();
        leaderClosedLoop = leader.getClosedLoopController();
        followerClosedLoop = follower.getClosedLoopController();
    }

    /**
     * Set shooter velocity in RPM.
     * @param rpm target velocity in rotations per minute
     */
    public void setVelocityRPM(double rpm) {
        leaderClosedLoop.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void setSpeed(double speed) {
        leader.set(speed);
    }

    /** Stop the shooter (velocity -> 0 RPM). */
    public void stop() {
        setVelocityRPM(0.0);
    }

    /**
     * @return current encoder velocity in RPM (matching ShooterConfigs velocityConversionFactor)
     */
    public double getVelocityRPM() {
        return encoder.getVelocity();
    }
    /**
     * @return target shooting speed in m/s
     */
    public double getShootingRPM() {
        return 1.0;
    }
    // public boolean fuelInShooter(){
    //     LaserCan.Measurement measurement;
    //     measurement = lc.getMeasurement();
    //     if (measurement != null){
    //         return  (measurement.distance_mm < 450);
    //     }
    //     else{
    //         return false;
    //     }
    // }

    public void periodic() {
        SmartDashboard.putNumber("ShooterRPM", getVelocityRPM());
        // SmartDashboard.putBoolean("Shooter/isFuelInShooter", fuelInShooter());
    }
}
