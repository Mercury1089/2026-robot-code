package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.BREAKBEAM;

import frc.robot.Constants;

public class Hopper {
    private SparkMax hopper;

    // Changed to 1 and 2 to FRONT and BACK for clarity
    private DigitalInput hopperBreakBeamFront;
    private DigitalInput hopperBreakBeamBack;
    public final int HOPPER_BREAKBEAM_FRONT = BREAKBEAM.HOPPER_BREAKBEAM_FRONT;
    public final int HOPPER_BREAKBEAM_BACK = BREAKBEAM.HOPPER_BREAKBEAM_BACK;

    public Hopper() {

        hopper = new SparkMax(Constants.CAN.HOPPER, MotorType.kBrushless);
        SparkMaxConfig hopperConfig = new SparkMaxConfig();

        hopperConfig.idleMode(IdleMode.kCoast)
                .inverted(true);

        hopper.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.hopperBreakBeamFront = new DigitalInput(HOPPER_BREAKBEAM_FRONT);
        this.hopperBreakBeamBack = new DigitalInput(HOPPER_BREAKBEAM_BACK);
    }

    public enum HopperSpeed {
        INDEX(1.0),
        STOP(0.0);

        public final double speed;

        HopperSpeed(double speed) {
            this.speed = speed;
        }
    }

    public void setSpeed(HopperSpeed intakeSpeed) {
        hopper.set(intakeSpeed.speed);
    }

    //!hopperBreakBeam1.get() is true if it is blocked (has fuel)
    public boolean frontHasFuel() {
        return !hopperBreakBeamFront.get();
    }

    public boolean backHasFuel() {
        return !hopperBreakBeamBack.get();
    }

    public boolean hasFuel() {
        return frontHasFuel() || backHasFuel();
    }

    public boolean hasPartialFuel() {
        return !(frontHasFuel() && backHasFuel()) && (hasFuel());
    }

    public boolean isFull() {
        return frontHasFuel() && backHasFuel();
    }
}
