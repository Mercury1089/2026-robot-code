// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TargetUtils;

public class RobotModeLEDs extends SubsystemBase {

  private Spark blinkin;
  private RobotMode robotMode;
  /** Creates a new GamePieceLEDs. */
  public RobotModeLEDs() {
    this.blinkin = new Spark(0);
    robotMode = RobotMode.FIDODISABLED;
  }

  /** set by the buttons
   * - saves the game state
   * - sets the SD boolean box to color
   * - sets the physical LED
  */

  public void enableFIDO() {
    this.robotMode = RobotMode.FIDOENABLED;
  }

  public void disableFIDO() {
    this.robotMode = RobotMode.FIDODISABLED;
  }

  public void toggleFIDO() {
    robotMode = robotMode == RobotMode.FIDOENABLED ?
                  RobotMode.FIDODISABLED :
                  RobotMode.FIDOENABLED;
  }

  public boolean isFIDOEnabled() {
    return this.robotMode == RobotMode.FIDOENABLED;
  }

  public enum LEDState {
    OFF(0.99),
    FIDO_ON_L4(-0.11), //strobe red
    FIDO_ON_L3(-0.09), //strobe blue
    FIDO_ON_L2(-0.07), //strobe gold
    FIDO_ON_L1(-0.05), //strobe white
    FIDO_OFF_L4(0.61), //red
    FIDO_OFF_L3(0.87), //blue
    FIDO_OFF_L2(0.67), //gold
    FIDO_OFF_L1(0.93); //white


    public final double colorValue;

    LEDState(double colorValue)  {
        this.colorValue = colorValue;
    }
  }

  public enum RobotMode {
    FIDOENABLED,
    FIDODISABLED
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED Color", robotMode.toString());
    SmartDashboard.putBoolean("LEDs/enableAutoShoot", isFIDOEnabled());
  }
}