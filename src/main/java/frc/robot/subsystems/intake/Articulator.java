// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.intake;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class Articulator extends SubsystemBase {

  /** Creates a new AlgaeArticulator. */
  public static final int
    ARM_PID_SLOT = 0;

  private static final double 
    ARM_NORMAL_P_VAL = 1.0 / 1.0,
    ARM_NORMAL_I_VAL = 0.0,
    ARM_NORMAL_D_VAL = 0.0;

  private static final float ARM_SOFT_LIMIT_FWD = (float) 120;

  private static final float ARM_SOFT_LIMIT_REV = (float) 50;

  private static final double ANGLE_OFFSET = -3.5;

  public final double GEAR_RATIO = 125.0 / 1.0;
  public final double THRESHOLD_DEGREES = 0.5;

  
  private SparkMax articulator;
  private SparkClosedLoopController articulatorClosedLoopController;
  private AbsoluteEncoder absoluteEncoder;
  private double setPosition;

  public Articulator() {
    articulator = new SparkMax(CAN.ARTICULATOR, MotorType.kBrushed);

    SparkMaxConfig articulatorConfig = new SparkMaxConfig();

    articulatorConfig
      .idleMode(IdleMode.kBrake)
      .inverted(true);
    articulatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(0.05,0,0)
      // .positionWrappingEnabled(true) we don't actually want this 
      // .positionWrappingInputRange(0.0, 360.0)
      .positionWrappingEnabled(false)
      .outputRange(-1,1);
    articulatorConfig.absoluteEncoder
      .positionConversionFactor(360.0)
      .zeroCentered(true); // -180 to 180
    articulatorConfig.softLimit
      .forwardSoftLimitEnabled(false)
      .forwardSoftLimit(320.0)
      .reverseSoftLimitEnabled(false)
      .reverseSoftLimit(191.0);
    
    articulator.configure(articulatorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    articulatorClosedLoopController = articulator.getClosedLoopController();

    absoluteEncoder = articulator.getAbsoluteEncoder();
    setPosition = getPosition();
  }
  
  public void resetEncoders() {
    articulatorClosedLoopController.setReference(0, SparkMax.ControlType.kPosition);
  }

  public void setSpeed(Supplier<Double> speedSupplier) {
    articulator.set((speedSupplier.get() * 0.5));
  }

  public void changePos() {
    setPosition(SmartDashboard.getNumber("Articulator/Position", 110.0));
  }

  public void setPosition(double pos) {
    setPosition = pos;
    articulatorClosedLoopController.setSetpoint(pos, SparkMax.ControlType.kPosition);
  }

  public void setPosition(ArticulatorPosition pos) {
    setPosition(pos.degreePos);
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
    SmartDashboard.putNumber("Articulator/Position", getPosition());
    SmartDashboard.putBoolean("Articulator/isInPosition", isInPosition());

  }
  
  public enum ArticulatorPosition {
    IN(0.0),
    MIDDLE(75.0),
    OUT(150.0);
    
    public final double degreePos;
      ArticulatorPosition(double degreePos) {
        this.degreePos = degreePos;
      }
  }
  

}