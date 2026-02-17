package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private SparkMax indexer;

  public Indexer() {

    indexer = new SparkMax(Constants.CAN.INDEXER, MotorType.kBrushless);
    SparkMaxConfig indexerConfig = new SparkMaxConfig();

    indexerConfig.idleMode(IdleMode.kBrake)
        .inverted(true);

    indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public enum IndexerSpeed {
    INDEX(1.0),
    STOP(0.0);

    public final double speed;

    IndexerSpeed(double speed) {
      this.speed = speed;
    }
  }

  public void setSpeed(IndexerSpeed indexerSpeed) {
    indexer.set(indexerSpeed.speed);
  }
}