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
    robotMode = RobotMode.AUTOSHOOTDISABLED;
  }

  public void enableAutoShoot() {
    robotMode = RobotMode.AUTOSHOOT;
  }

  public void disableAutoShoot() {
    robotMode = RobotMode.AUTOSHOOTDISABLED;
  }

  public boolean isAutoShootEnabled() {
    return robotMode == RobotMode.AUTOSHOOT;
  }

  public void toggleAutoShoot() {
    robotMode = robotMode == RobotMode.AUTOSHOOT ? RobotMode.AUTOSHOOTDISABLED : RobotMode.AUTOSHOOT;
  }

  /** set by the buttons
   * - saves the game state
   * - sets the SD boolean box to color
   * - sets the physical LED
  */


  public enum LEDState {
    OFF(0.99);


    public final double colorValue;

    LEDState(double colorValue)  {
        this.colorValue = colorValue;
    }
  }

  public enum RobotMode {
    AUTOSHOOT,
    AUTOSHOOTDISABLED
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED Color", robotMode.toString());
  }
}