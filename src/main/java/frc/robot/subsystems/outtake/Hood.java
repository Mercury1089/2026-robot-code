package frc.robot.subsystems.outtake;

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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.intake.Articulator.ArticulatorPosition;

public class Hood extends SubsystemBase {
    public final double THRESHOLD_DEGREES = 0.5;

    private SparkMax hood;
    private SparkClosedLoopController hoodClosedLoopController;
    private AbsoluteEncoder absoluteEncoder;
    private double setPosition;
    
    public Hood() {
        hood = new SparkMax(CAN.HOOD, MotorType.kBrushed);

        SparkMaxConfig hoodConfig = new SparkMaxConfig();

        hoodConfig 
                .idleMode(IdleMode.kBrake)
                .inverted(true);
        hoodConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(0.1, 0, 0)
                // .positionWrappingEnabled(true) we don't actually want this
                // .positionWrappingInputRange(0.0, 360.0)
                .positionWrappingEnabled(false)
                .outputRange(-1, 1);
        hoodConfig.absoluteEncoder
                .positionConversionFactor(360.0)
                .zeroCentered(true); // -180 to 180
        hoodConfig.softLimit
                .forwardSoftLimitEnabled(false)
                .forwardSoftLimit(320.0)
                .reverseSoftLimitEnabled(false)
                .reverseSoftLimit(191.0);

        hood.configure(hoodConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        hoodClosedLoopController = hood.getClosedLoopController();

        absoluteEncoder = hood.getAbsoluteEncoder();
        setPosition = getPosition();
    }

    public void resetEncoders() {
        hoodClosedLoopController.setSetpoint(0, SparkMax.ControlType.kPosition);
    }

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

    public void setPosition(ArticulatorPosition pos) {
        setPosition(pos.degreePos);
    }

    public double getHoodToHubPosition() {
        return 0.0;
    }

    public boolean isAtPosition(double pos) {
        return Math.abs(getPosition() - pos) < THRESHOLD_DEGREES;
    }

    public boolean isAtPosition(ArticulatorPosition pos) {
        return Math.abs(getPosition() - pos.degreePos) < THRESHOLD_DEGREES;
    }

    public double getPosition() {
        return absoluteEncoder.getPosition();
    }

    public boolean isInPosition() {
        return isAtPosition(setPosition);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Hood/Position", getPosition());
        SmartDashboard.putBoolean("Hood/isInPosition", isInPosition());

    }
    
}
