package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;

import frc.robot.Constants;

/** Shooter subsystem with two NEO motors controlled by SPARK MAX closed-loop velocity control. */
public class Shooter extends SubsystemBase {
    private final SparkMax leader;
    private final SparkMax follower;

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController leaderClosedLoop;
    private final SparkClosedLoopController followerClosedLoop;

    public Shooter() {
        leader = new SparkMax(Constants.CAN.SHOOTER, MotorType.kBrushless);
        follower = new SparkMax(Constants.CAN.SHOOTER_BACK_SPARKMAX, MotorType.kBrushless);

        // Build the shooter SparkMaxConfig inline (previously in ShooterConfigs)
        SparkMaxConfig leader_config = new SparkMaxConfig();
        leader_config.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
        // use RPM units for encoder velocity here (1.0 = pass-through)
        leader_config.encoder.velocityConversionFactor(1.0);

        double kP = 0.0006;
        double kI = 0.0;
        double kD = 0.0;
        // TODO: The velocityFF needs to be fixed
        double velocityFF = 1.0 / Constants.NEO_MOTOR_CONSTANTS.FREE_SPEED_RPMS;

        leader_config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .velocityFF(velocityFF)
            .outputRange(-1.0, 1.0);

        // Build the shooter SparkMaxConfig inline (previously in ShooterConfigs)
        SparkMaxConfig follower_config = new SparkMaxConfig();
        follower_config.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
        // use RPM units for encoder velocity here (1.0 = pass-through)
        follower_config.encoder.velocityConversionFactor(1.0);
        follower_config.follow(Constants.CAN.SHOOTER_BACK_SPARKMAX);

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
        followerClosedLoop.setSetpoint(rpm, ControlType.kVelocity);
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
}
