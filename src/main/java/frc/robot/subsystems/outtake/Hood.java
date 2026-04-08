package frc.robot.subsystems.outtake;

import java.lang.annotation.Target;
import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.Zone;
import frc.robot.subsystems.intake.Articulator.ArticulatorPosition;
import frc.robot.util.KnownLocations;
import frc.robot.util.TargetUtils;

public class Hood extends SubsystemBase {
    public final double THRESHOLD_DEGREES = 10.0;
    private final double RADIANSTOHOODANGLE = 100;

    private SparkMax hood;
    private SparkClosedLoopController hoodClosedLoopController;
    private AbsoluteEncoder absoluteEncoder;
    private double setPosition;
    
    private Drivetrain drivetrain;
    
    public Hood(Drivetrain drivetrain) {
        hood = new SparkMax(CAN.HOOD, MotorType.kBrushed);

        this.drivetrain = drivetrain;

        SparkMaxConfig hoodConfig = new SparkMaxConfig();

        hoodConfig 
                .idleMode(IdleMode.kBrake)
                .inverted(true);
        hoodConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1.0 / 20.0, 0, 0)
                // .positionWrappingEnabled(true) we don't actually want this
                // .positionWrappingInputRange(0.0, 360.0)
                .positionWrappingEnabled(false)
                .outputRange(-1, 1);
        hoodConfig.absoluteEncoder
                .positionConversionFactor(360.0)
                .zeroCentered(true); // -180 to 180
        hoodConfig.softLimit//TODO: tweak rotation values as necessary, potentially disable zeroCentered
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(75.0)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(-160.0);//consider making this a little less

        hood.configure(hoodConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        hoodClosedLoopController = hood.getClosedLoopController();

        absoluteEncoder = hood.getAbsoluteEncoder();
        setPosition = getPosition();
        // desiredSetPosition = getPosition();
    }

    public double getSetPosition() {
        return setPosition;
    }

    public void goToSetPosition() {
        setPosition(setPosition);
        // setPosition(getHoodToFirePosition());
    }

    public void resetEncoders() {
        hoodClosedLoopController.setSetpoint(0, SparkMax.ControlType.kPosition);
    }

    //same weird stuff as articulator
    public void setSpeed(Supplier<Double> speedSupplier) {
        hood.set((speedSupplier.get() * 0.5));
    }

    public void changePos() {
        setPosition(SmartDashboard.getNumber("Hood/Position", 110.0));
    }

    public void setPosition(double pos) {
        setPosition = pos;
        hoodClosedLoopController.setSetpoint(pos, SparkMax.ControlType.kPosition);
    }

    public void minusOneDegree() {
        setPosition = setPosition - 1.0;
    }  

    public void plusOneDegree() {
        setPosition = setPosition + 1.0;
    }

    public double getHoodToFirePosition(boolean passing) {
        Translation2d point = new Translation2d();

        if(drivetrain.isDrivetrainInAllianceZone() || drivetrain.getCurrentZone() == Zone.BETWEEN) {
            point = KnownLocations.getKnownLocations().HUB.getTranslation();
        } else if(drivetrain.getCurrentZone() == Zone.NEUTRAL_LEFT) {
            point = KnownLocations.getKnownLocations().PASSING_TARGET_LEFT.getTranslation();
        } else if(drivetrain.getCurrentZone() == Zone.NEUTRAL_RIGHT) {
            point = KnownLocations.getKnownLocations().PASSING_TARGET_RIGHT.getTranslation();
        }

        if(drivetrain.isDrivetrainInAllianceZone()) {
            return HoodPosition.SHOOT.pos; // shooting function
        } else {
            if (passing) {
                return HoodPosition.FERRY.pos;
            }
            return HoodPosition.SHOOT.pos; // passing function
        }
    }

    // public void setHoodToFirePosition() {
    //     setPosition(getHoodToFirePosition(false));
    // }

    public boolean isAtPosition(double pos) {
        return Math.abs(getPosition() - pos) < THRESHOLD_DEGREES;
    }

    public double getPosition() {
        return absoluteEncoder.getPosition();
    }

    public boolean isInPosition() {
        return isAtPosition(setPosition);
    }

    @Override
    public void periodic() {
        // desiredSetPosition = getHoodToFirePosition();
        
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Hood/hoodPosition", getPosition());
        SmartDashboard.putBoolean("Hood/hoodIsInPosition", isInPosition());

    }
    
    public enum HoodPosition {
        FERRY(-22.0),
        SHOOT(-160.0);//the one we used for our equations for

        public final double pos;

        HoodPosition(double pos) {
            this.pos = pos;
        }
    }
}
