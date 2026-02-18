package frc.robot.subsystems.outtake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class Kicker extends SubsystemBase{
    private SparkMax kicker;

  public Kicker() {

    kicker = new SparkMax(Constants.CAN.KICKER, MotorType.kBrushless);
    SparkMaxConfig kickerConfig = new SparkMaxConfig();

    kickerConfig.idleMode(IdleMode.kCoast)
        .inverted(true);

    kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

    public enum KickerSpeed {
        INDEX(1.0),
        STOP(0.0);

        public final double speed;

        KickerSpeed(double speed) {
            this.speed = speed;
        }
    }

    public void setSpeed(KickerSpeed intakeSpeed) {
        kicker.set(intakeSpeed.speed);
    }
}