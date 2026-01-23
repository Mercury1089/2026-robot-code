package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private SparkFlex intake;
    private final int INTAKE_CURRENT_LIMIT = 18;
        
    public Intake() {

        intake = new SparkFlex(Constants.CAN.INTAKE, MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(INTAKE_CURRENT_LIMIT);

        intakeConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kNoSensor)
        .pid(0.5,0.0,0.0);

        intake.configure(intakeConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
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