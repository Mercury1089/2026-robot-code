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

  public final double THRESHOLD_DEGREES = 5.0;

  private SparkMax articulator;
  private SparkClosedLoopController articulatorClosedLoopController;
  private AbsoluteEncoder absoluteEncoder;
  private double setPosition;

  public Articulator() {
    articulator = new SparkMax(CAN.ARTICULATOR, MotorType.kBrushed);

    SparkMaxConfig articulatorConfig = new SparkMaxConfig();

    articulatorConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    articulatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(1.0 / 20.0, 0, 0)
      // .positionWrappingEnabled(true) we don't actually want this 
      // .positionWrappingInputRange(0.0, 360.0)
      .positionWrappingEnabled(false)
      .outputRange(-0.6,1);
    articulatorConfig.absoluteEncoder
      .positionConversionFactor(360.0)
      .zeroCentered(true); // -180 to 180 
    articulatorConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(50.0)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(-32.0);
    
    articulator.configure(articulatorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    articulatorClosedLoopController = articulator.getClosedLoopController();

    absoluteEncoder = articulator.getAbsoluteEncoder();
    setPosition = getPosition();
  }
  
  public void resetEncoders() {
    articulatorClosedLoopController.setSetpoint(0, SparkMax.ControlType.kPosition);
  }

  public void setSpeed(Supplier<Double> speedSupplier) {//why do we have this
    articulator.set((speedSupplier.get() * .8));
  }

  public void changePos() {//???
    setPosition(SmartDashboard.getNumber("Articulator/Position", 110.0));
  }

  public void setPosition(double pos) {
    setPosition = pos;
    articulatorClosedLoopController.setSetpoint(pos, SparkMax.ControlType.kPosition);
  }

  public void setPosition(ArticulatorPosition pos) {
    setPosition(pos.degreePos);
  }

  public boolean canIntake() {
    return getPosition() < (ArticulatorPosition.CAN_INTAKE.degreePos);
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
    SmartDashboard.putNumber("Articulator/articulatorPosition", getPosition());
    SmartDashboard.putBoolean("Articulator/isInPosition", isInPosition());

  }
  
  public enum ArticulatorPosition {//TODO: tweak rotation values as necessary, potentially disable zeroCentered
    IN(47.0),
    SAFE(0.0),
    CAN_INTAKE(-20.0),
    OUT(-32.0);
    
    public final double degreePos;
      ArticulatorPosition(double degreePos) {
        this.degreePos = degreePos;
      }
  }

}