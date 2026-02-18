// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DS_USB;
import frc.robot.Constants.JOYSTICK_BUTTONS;
import frc.robot.commands.Autons;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.RobotModeLEDs;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Articulator.ArticulatorPosition;
import frc.robot.subsystems.intake.Articulator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeSpeed;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.util.KnownLocations;
import frc.robot.util.TargetUtils;
import frc.robot.util.Shift;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.TriggerEvent;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; 
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private CommandJoystick rightJoystick, leftJoystick;
  private CommandXboxController gamepad;
  // private CommandGenericHID reefBoard;
  // private CommandGenericHID secondEncoderBoard;

  private Trigger left1, left2, left3, left4, left5, left6, left7, left8, left9, left10, left11;
  private Trigger right1, right2, right3, right4, right5, right6, right7, right8, right9, right10, right11;
  private Trigger gamepadA, gamepadB, gamepadX, gamepadY, gamepadRB, gamepadLB, gamepadL3, gamepadBack, 
  gamepadStart, gamepadLeftStickButton, gamepadRightStickButton, gamepadLT, gamepadRT, gamepadPOVDown, gamepadPOVUpLeft, 
  gamepadPOVUp, gamepadPOVUpRight, gamepadPOVLeft, gamepadPOVRight, gamepadPOVDownRight, gamepadPOVDownLeft;

  private GenericHID gamepadHID;
  private Supplier<Double> gamepadLeftX, gamepadLeftY, gamepadRightX, gamepadRightY;
  private Supplier<Double> rightJoystickX, rightJoystickY, leftJoystickX, leftJoystickY;

  private Autons auton;
  private Drivetrain drivetrain;
  private Shooter shooter;
  private Articulator articulator;
  private RobotModeLEDs leds;
  private Shift shift;

  private double manualThreshold = 0.2;
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // buttons & bindings
    leftJoystick = new CommandJoystick(DS_USB.LEFT_STICK);
    rightJoystick = new CommandJoystick(DS_USB.RIGHT_STICK);
    gamepad = new CommandXboxController(DS_USB.GAMEPAD);
    // reefBoard = new CommandGenericHID(DS_USB.REEF_BOARD);
    // secondEncoderBoard = new CommandGenericHID(DS_USB.SECOND_ENCODER_BOARD);
    gamepadHID = new GenericHID(DS_USB.GAMEPAD);
    configureBindings();

    shooter = new Shooter();
    drivetrain = new Drivetrain(shooter);
    drivetrain.setDefaultCommand(DriveCommands.joyStickDrive(leftJoystickY, leftJoystickX, rightJoystickX, drivetrain));
    drivetrain.resetGyro();

    Intake intake = new Intake();
    intake.setDefaultCommand(new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake));
    Hopper hopper = new Hopper();
    
    auton = new Autons(drivetrain);

    leds = new RobotModeLEDs();

    /**
     * MANUAL CONTROL
     */
    left10.onTrue(new InstantCommand(() -> drivetrain.resetGyro(), drivetrain).ignoringDisable(true));
    left11.onTrue(new InstantCommand(() -> drivetrain.recalibrateGyro(), drivetrain).ignoringDisable(true));
    right2.onTrue(drivetrain.getDefaultCommand());
    
    articulator = new Articulator();
    articulator.setDefaultCommand(new RunCommand(() -> articulator.setSpeed(gamepadLeftY), articulator));
    gamepadPOVLeft.onTrue(new RunCommand(() -> articulator.setPosition(ArticulatorPosition.IN), articulator));
    gamepadPOVUp.onTrue(new RunCommand(() -> articulator.setPosition(ArticulatorPosition.SAFE), articulator));
    gamepadPOVRight.onTrue(new RunCommand(() -> articulator.setPosition(ArticulatorPosition.OUT), articulator));

    shooter.setDefaultCommand(new RunCommand(() -> shooter.stop(), shooter));
    // gamepadA.onTrue(new RunCommand(() -> shooter.setVelocityRPM(1500.0), shooter));
    // // gamepadA.onTrue(new RunCommand(() -> shooter.setSpeed(1.0), shooter));
    // gamepadB.onTrue(new RunCommand(() -> shooter.stop(), shooter));
    // gamepadY.onTrue(new RunCommand(() -> shooter.setVelocityRPM(5000.0), shooter));
    // gamepadX.onTrue(new RunCommand(() -> shooter.setVelocityRPM(3000.0), shooter));
    
    gamepadA.and(() -> DriverStation.getGameSpecificMessage().isBlank()).onTrue(
      new InstantCommand(() -> drivetrain.getShift().setManualAutonWinner("R")) 
    );

    gamepadB.and(() -> DriverStation.getGameSpecificMessage().isBlank()).onTrue(
      new InstantCommand(() -> drivetrain.getShift().setManualAutonWinner("B")) 
    );


    // left3.whileTrue(new SequentialCommandGroup(
    //   new InstantCommand(() -> drivetrain.setXDirStraight(0.1), drivetrain),
    //   new InstantCommand(() -> drivetrain.setYDirStraight(0.1), drivetrain),
    //   new InstantCommand(() -> drivetrain.setRotationBeforeStraightDrive(drivetrain.getRotation()), drivetrain),
    //   DriveCommands.driveStraightAtAngle(() -> drivetrain.getHeadingToHub(), drivetrain)
    // ));

    left3.whileTrue(DriveCommands.shootOnTheMove(leftJoystickY, leftJoystickX, drivetrain));

    left2.onTrue(DriveCommands.safelyDriveOverBump(leftJoystickY, leftJoystickX, drivetrain));

    right8.onTrue(DriveCommands.lockToHub(leftJoystickY, leftJoystickX, drivetrain));

    right1.whileTrue(new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake));
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        left1 = leftJoystick.button(JOYSTICK_BUTTONS.BTN1);
        left2 = leftJoystick.button(JOYSTICK_BUTTONS.BTN2);
        left3 = leftJoystick.button(JOYSTICK_BUTTONS.BTN3);
        left4 = leftJoystick.button(JOYSTICK_BUTTONS.BTN4);
        left5 = leftJoystick.button(JOYSTICK_BUTTONS.BTN5);
        left6 = leftJoystick.button(JOYSTICK_BUTTONS.BTN6);
        left7 = leftJoystick.button(JOYSTICK_BUTTONS.BTN7);
        left8 = leftJoystick.button(JOYSTICK_BUTTONS.BTN8);
        left9 = leftJoystick.button(JOYSTICK_BUTTONS.BTN9);
        left10 = leftJoystick.button(JOYSTICK_BUTTONS.BTN10);
        left11 = leftJoystick.button(JOYSTICK_BUTTONS.BTN11);

        right1 = rightJoystick.button(JOYSTICK_BUTTONS.BTN1);
        right2 = rightJoystick.button(JOYSTICK_BUTTONS.BTN2);
        right3 = rightJoystick.button(JOYSTICK_BUTTONS.BTN3);
        right4 = rightJoystick.button(JOYSTICK_BUTTONS.BTN4);
        right5 = rightJoystick.button(JOYSTICK_BUTTONS.BTN5);
        right6 = rightJoystick.button(JOYSTICK_BUTTONS.BTN6);
        right7 = rightJoystick.button(JOYSTICK_BUTTONS.BTN7);
        right8 = rightJoystick.button(JOYSTICK_BUTTONS.BTN8);
        right9 = rightJoystick.button(JOYSTICK_BUTTONS.BTN9);
        right10 = rightJoystick.button(JOYSTICK_BUTTONS.BTN10);
        right11 = rightJoystick.button(JOYSTICK_BUTTONS.BTN11);

        gamepadA = gamepad.a();
        gamepadB = gamepad.b();
        gamepadX = gamepad.x();
        gamepadY = gamepad.y();
        gamepadRB = gamepad.rightBumper();
        gamepadLB = gamepad.leftBumper();
        gamepadBack = gamepad.back();
        gamepadStart = gamepad.start();
        gamepadLeftStickButton = gamepad.leftStick();
        gamepadRightStickButton = gamepad.rightStick();
        gamepadLT = gamepad.leftTrigger();
        gamepadRT = gamepad.rightTrigger();
        
        gamepadPOVDown = gamepad.povDown();
        gamepadPOVUpLeft = gamepad.povUpLeft();
        gamepadPOVUp = gamepad.povUp();
        gamepadPOVUpRight = gamepad.povUpRight();
        gamepadPOVLeft = gamepad.povLeft();
        gamepadPOVRight = gamepad.povRight();
        gamepadPOVDownRight = gamepad.povDownRight();
        gamepadPOVDownLeft = gamepad.povDownLeft();

        gamepadLeftX = () -> gamepad.getLeftX();
        gamepadRightX = () -> gamepad.getRightX();
        gamepadLeftY = () -> -gamepad.getLeftY();
        gamepadRightY = () -> -gamepad.getRightY();

        leftJoystickX = () -> leftJoystick.getX();
        leftJoystickY = () -> leftJoystick.getY();
        rightJoystickX = () -> rightJoystick.getX();
        rightJoystickY = () -> rightJoystick.getY();


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Autons getAutonomous() {
    // An example command will be run in autonomous
    return auton;
  }
}
