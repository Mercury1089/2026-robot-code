package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkFlex intake;

  public Intake() {

    intake = new SparkFlex(Constants.CAN.INTAKE, MotorType.kBrushless);
    SparkFlexConfig intakeConfig = new SparkFlexConfig();

    intakeConfig.idleMode(IdleMode.kBrake)
        .inverted(true);

    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public enum IntakeSpeed {
    INTAKE(1.0),
    STOP(0.0);

    public final double speed;

    IntakeSpeed(double speed) {
      this.speed = speed;
    }
  }

  public void setSpeed(IntakeSpeed intakeSpeed) {
    intake.set(intakeSpeed.speed);
  }
}