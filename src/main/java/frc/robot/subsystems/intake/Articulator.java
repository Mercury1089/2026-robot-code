// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.articulator;

import java.time.format.TextStyle;
import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.TargetUtils;


public class Articulator extends SubsystemBase {
  /** Creates a new Arm. */
  public static final int
    ARM_PID_SLOT = 0;

  private static final double 
    ARM_NORMAL_P_VAL = 1.0 / 10.0,
    ARM_NORMAL_I_VAL = 0.0,
    ARM_NORMAL_D_VAL = 0.0;

  private static final float ARM_SOFT_LIMIT_FWD = (float) 147;

  private static final float ARM_SOFT_LIMIT_BKW = (float) 45.3;

  private static final double ANGLE_OFFSET = -3.5;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.01, //0.02,
    PEAK_OUTPUT_FORWARD = 1.0, // 0.6,
    NOMINAL_OUTPUT_REVERSE = -0.01, //-0.5,
    PEAK_OUTPUT_REVERSE = -0.6;

  public final double GEAR_RATIO = 125.0 / 1.0;
  public final double THRESHOLD_DEGREES = 0.5;

  
  private SparkMax articulator;
  private AbsoluteEncoder armAbsoluteEncoder;
  private Drivetrain drivetrain;
  private double setPosition;

  public Articulator(Drivetrain drivetrain) {
    articulator = new SparkMax(CAN.ARTICULATOR, MotorType.kBrushed);

    SparkMaxConfig config = new SparkMaxConfig();
// TODO: finish this its like a third finished lmao
    double kP = 0.0006;
    double kI = 0.0;
    double kD = 0.0;

    config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD);

    // config.absoluteEncoder = articulator.getAbsoluteEncoder();
    config.idleMode(IdleMode.kBrake);
    // armAbsoluteEncoder = armLeft.getAbsoluteEncoder(Type.kDutyCycle);

    // armAbsoluteEncoder.setPositionConversionFactor(360.0);
    // armPIDController.setFeedbackDevice(armAbsoluteEncoder);

    // armPIDController.setPositionPIDWrappingEnabled(false);

    setPosition = getArmPosition();

    SmartDashboard.putNumber("Articulator/Position", getArmPosition());
  }
  
  public void resetEncoders() {
    articulatorPIDController.setReference(0, SparkMax.ControlType.kPosition);
  }

  public void setSpeed(Supplier<Double> speedSupplier) {
    articulator.set((speedSupplier.get() * 0.5));
  }

  public void setPosition(ArticulatorPosition pos) {
    setPosition(pos.degreePos);
  }

  public void changePos() {
    setPosition(SmartDashboard.getNumber("Articulator/Position", 110.0));
  }

  public void setPosition(double pos) {
    setPosition = pos;

    armPIDController.setReference(pos, SparkMax.ControlType.kPosition);
  }
  
  public boolean isAtPosition(double pos) {
    return Math.abs(getArmPosition() - pos) < THRESHOLD_DEGREES;
  }

  public boolean isAtPosition(ArticulatorPosition pos) {
    return isAtPosition(pos.degreePos);
  }

  public double getArmPosition() {
    return armAbsoluteEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm/Encoder", getArmPosition());
  }
  

  public enum ArticulatorPosition {
    OUT(150.0),
    IN(ARM_SOFT_LIMIT_BKW);
  
    
    public final double degreePos;
      ArticulatorPosition(double degreePos) {
        this.degreePos = degreePos;
      }
  }

}