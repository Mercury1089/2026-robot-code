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


  public enum LEDState {
    OFF(0.99);


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
  }
}