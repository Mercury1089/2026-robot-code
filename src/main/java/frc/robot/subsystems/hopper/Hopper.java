package frc.robot.subsystems.hopper;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.BREAKBEAM;
import frc.robot.sensors.ProximitySensor;
import frc.robot.Constants;

public class Hopper {
    private SparkMax hopper;
    private LaserCan lc;
    private ProximitySensor proxSensor;
    // Changed to 1 and 2 to FRONT and BACK for clarity
    // private DigitalInput hopperBreakBeamFront;
    //private DigitalInput hopperBreakBeamBack;
    // public final int HOPPER_BREAKBEAM_FRONT = BREAKBEAM.HOPPER_BREAKBEAM_FRONT;
    // public final int HOPPER_BREAKBEAM_BACK = BREAKBEAM.HOPPER_BREAKBEAM_BACK;

    public Hopper() {

        hopper = new SparkMax(Constants.CAN.HOPPER, MotorType.kBrushless);
        SparkMaxConfig hopperConfig = new SparkMaxConfig();

        hopperConfig.idleMode(IdleMode.kCoast)
                .inverted(true);

        hopper.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // this.hopperBreakBeamFront = new DigitalInput(HOPPER_BREAKBEAM_FRONT);
        // this.hopperBreakBeamBack = new DigitalInput(HOPPER_BREAKBEAM_BACK);

        lc = new LaserCan(Constants.CAN.LASER_CAN);
        proxSensor = new ProximitySensor(Constants.CAN.PROX_SENSOR, 0.1);

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
    // public boolean frontHasFuel() {
    //     return !hopperBreakBeamFront.get();
    // }

    // public boolean backHasFuel() {
    //     // return !hopperBreakBeamBack.get();
    // }

    // public boolean hasFuel() {
    //     return frontHasFuel() || backHasFuel();
    // }

    // public boolean hasPartialFuel() {
    //     return !(frontHasFuel() && backHasFuel()) && (hasFuel());
    // }

    // public boolean isFull() {
    //     return frontHasFuel() && backHasFuel();
    // }

    public void periodic() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            SmartDashboard.putNumber("Hopper/LaserCanMeasurement", measurement.distance_mm);
        }
        SmartDashboard.putBoolean("Hopper/ProxSensorIsTriggered", proxSensor.isTriggered());
    }
}
