// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.KnownLocations;
import com.reduxrobotics.canand.CanandEventLoop;

import au.grapplerobotics.CanBridge;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;
  private Alliance m_alliance;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CanBridge.runTCP();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    
    m_robotContainer = new RobotContainer();
    CanandEventLoop.getInstance();
    m_alliance = KnownLocations.getKnownLocations().alliance;
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.getAutonomous().updateDash();

    if (m_alliance != KnownLocations.getKnownLocations().alliance) {
      // m_robotContainer.initializeTriggers();
      m_robotContainer.getAutonomous().updateAutonLocations();

      m_alliance = KnownLocations.getKnownLocations().alliance;
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Command autonCommand = m_robotContainer.getAutonomous().getAutonCommand();
    m_robotContainer.getDrivetrain().resetPose(m_robotContainer.getDrivetrain().getPose()); // check this and if we want it
    if (autonCommand != null){
      CommandScheduler.getInstance().schedule(autonCommand);
      DriverStation.reportError("Auton is Scheduled", false);
    }
    DriverStation.reportError("Auton is initialized", false);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // m_robotContainer.getCoralIntake().setEjecting(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    CommandScheduler.getInstance().cancelAll();
  }
}
