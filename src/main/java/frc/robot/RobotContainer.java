// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DS_USB;
import frc.robot.Constants.JOYSTICK_BUTTONS;
import frc.robot.commands.Autons;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.RobotModeLEDs;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hopper.Indexer;
import frc.robot.subsystems.hopper.Indexer.IndexerSpeed;
import frc.robot.subsystems.intake.Articulator;
import frc.robot.subsystems.intake.Articulator.ArticulatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeSpeed;
import frc.robot.subsystems.outtake.Hood;
import frc.robot.subsystems.outtake.Kicker;
import frc.robot.subsystems.outtake.Kicker.KickerSpeed;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.util.Shift;


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
  private Intake intake;
  private Kicker kicker;
  private Indexer indexer;
  private Hood hood;

  private double manualThreshold = 0.2;
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // buttons & bindings
    leftJoystick = new CommandJoystick(DS_USB.LEFT_STICK);
    rightJoystick = new CommandJoystick(DS_USB.RIGHT_STICK);
    gamepad = new CommandXboxController(DS_USB.GAMEPAD);
    gamepadHID = new GenericHID(DS_USB.GAMEPAD);
    configureBindings();

    drivetrain = new Drivetrain();
    drivetrain.resetGyro();

    intake = new Intake();

    shooter = new Shooter(drivetrain);
    drivetrain.setShooter(shooter);
    // In the final code, the default should always be  
    // setting shooter speed to getStaticShootingRPM()
    // shooter.setDefaultCommand(new RunCommand(() -> shooter.stop(), shooter));
    // shooter.setDefaultCommand(new RunCommand(() -> shooter.setVelocityRPM(shooter.getStaticShootingRPM()), shooter));

    hood = new Hood(drivetrain);
    // In the final code, the default should always be setting hood position to getHoodToFirePosition()
    // hood.setDefaultCommand(new RunCommand(() -> hood.setPosition(hood.getHoodToFirePosition()), hood));
    // hood.setDefaultCommand(new RunCommand(() -> hood.setPosition(0.0), hood));

    kicker = new Kicker();

    indexer = new Indexer();

    articulator = new Articulator();

    leds = new RobotModeLEDs(drivetrain);
    
    auton = new Autons(drivetrain, hood, shooter, indexer, kicker, articulator);

    Map<String, Command> commands = new HashMap<String, Command>();

    commands.put("intake", RobotCommands.intake(intake, articulator));
    commands.put("stopIntake", RobotCommands.stopIntake(intake, articulator));
    // commands.put("shoot", RobotCommands.fireAuton(shooter, kicker, hood, indexer, articulator, hopper));
    //should stop shooting after 3 seconds
    // commands.put("stopShooting", RobotCommands.stopFire(shooter, kicker, articulator, indexer));
    
    NamedCommands.registerCommands(commands);
    /**
     * DEFAULT COMMANDS - NEED TO BE CREATED AFTER NAMED COMMANDS AS PER PATHPLANNERDOCS
     */
     drivetrain.setDefaultCommand(DriveCommands.joyStickDrive(leftJoystickY, leftJoystickX, rightJoystickX, drivetrain));
    //drivetrain.setDefaultCommand(DriveCommands.targetDrive(leftJoystickY, leftJoystickX,  () -> drivetrain.getFinalHeading(), drivetrain));
    
    intake.setDefaultCommand(new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake));
    
    // shooter.setDefaultCommand(new RunCommand(() -> shooter.setVelocityRPM(1000.0), shooter));
    //shooter.setDefaultCommand(new RunCommand(() -> shooter.setVelocityRPM(shooter.getSmartDashRPM()), shooter)); //tis one
    // shooter.setDefaultCommand(new RunCommand(() -> shooter.goToSetRPM(), shooter));
    // shooter.setDefaultCommand(new RunCommand(() -> shooter.stop(), shooter));
    // Uncomment below for final robot
    shooter.setDefaultCommand(new RunCommand(() -> shooter.setVelocityRPM(shooter.getStaticShootingRPM(false)), shooter));
    
    // hood.setDefaultCommand(new RunCommand(() -> hood.setSpeed(() -> 0.0), hood));
    // Uncomment below for final robot
    hood.setDefaultCommand(new RunCommand(() -> hood.setPosition(hood.getHoodToFirePosition(false)), hood));
    // hood.setDefaultCommand(new RunCommand(() -> hood.setSpeed(gamepadRightY), hood));
    // hood.setDefaultCommand(new RunCommand(() -> hood.goToSetPosition(), hood));
    
    kicker.setDefaultCommand(new RunCommand(() -> kicker.setSpeed(KickerSpeed.STOP), kicker));

    indexer.setDefaultCommand(new RunCommand(() -> indexer.setSpeed(IndexerSpeed.STOP), indexer));

    articulator.setDefaultCommand(new RunCommand(() -> articulator.setPosition(ArticulatorPosition.SAFE), articulator));
    // articulator.setDefaultCommand(new RunCommand(() -> articulator.setSpeed(gamepadRightY), articulator));

    /**
     * MANUAL CONTROL
     */
    left10.onTrue(new InstantCommand(() -> drivetrain.resetGyro(), drivetrain).ignoringDisable(true));
    left11.onTrue(new InstantCommand(() -> drivetrain.recalibrateGyro(), drivetrain).ignoringDisable(true));
    right2.onTrue(drivetrain.getDefaultCommand());
    
    gamepadPOVLeft.onTrue(new RunCommand(() -> articulator.setPosition(ArticulatorPosition.IN), articulator));
    gamepadPOVUp.onTrue(new RunCommand(() -> articulator.setPosition(ArticulatorPosition.SAFE), articulator));
    gamepadPOVRight.onTrue(new RunCommand(() -> articulator.setPosition(ArticulatorPosition.OUT), articulator));
      
    // gamepadA.onTrue(new InstantCommand(() -> drivetrain.getShift().setManualAutonWinner("R")));
    // gamepadB.onTrue(new InstantCommand(() -> drivetrain.getShift().setManualAutonWinner("B")));


    // Trigger shooting = new Trigger(() -> shooter.isAtShootingRPM());
    

    right8.whileTrue(new RunCommand(() -> indexer.setSpeed(IndexerSpeed.INDEX), indexer));
    right9.whileTrue(new RunCommand(() -> kicker.setSpeed(KickerSpeed.INDEX), kicker));

    // gamepadA.whileTrue(new RunCommand(() -> shooter.setSpeed(0.25), shooter));
    // gamepadB.whileTrue(new RunCommand(() -> shooter.setVelocityRPM(720.0), shooter));
    gamepadA.onTrue(new InstantCommand(() -> shooter.increaseRPM(), shooter));
    gamepadB.onTrue(new InstantCommand(() -> shooter.decreaseRPM(), shooter));
    // gamepadPOVDown.whileTrue(new RunCommand(() -> hood.setSpeed(gamepadRightY), hood));
    // gamepadRightStickButton.onTrue(new InstantCommand(() -> shooter.setVelocityRPM(3000), shooter));

    // gamepadY.whileTrue(new RunCommand(() -> hood.setPosition(ArticulatorPosition.OUT), hood)); put back later
    // gamepadX.onTrue(new InstantCommand(() -> hood.plusOneDegree(), hood));
    // gamepadY.onTrue(new InstantCommand(() -> hood.minusOneDegree(), hood));

    gamepadLB.onTrue(new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake));
    gamepadRB.onTrue(new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake));

    gamepadY.whileTrue(new RunCommand(() -> hood.setSpeed(gamepadLeftY), hood));

    // gamepadA.whileTrue(new RunCommand(() -> shooter.setVelocityRPM(2800), shooter));
    // gamepadB.whileTrue(new RunCommand(() -> indexer.setSpeed(IndexerSpeed.INDEX), indexer));
    // gamepadX.whileTrue(new RunCommand(() -> kicker.setSpeed(KickerSpeed.INDEX), kicker));

    // right1.onTrue(new InstantCommand(() -> leds.toggleAutoShoot(), leds));

    // gamepadLeftStickButton.whileTrue(RobotCommands.agitateIntake(articulator));

    // right1.whileTrue(new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake));

    /**
     * DRIVETRAIN AUTOMATION
     */
    left3.whileTrue(DriveCommands.shootOnTheMove(leftJoystickY, leftJoystickX, drivetrain));
    
    // left2.onTrue(DriveCommands.safelyDriveOverBump(leftJoystickY, leftJoystickX, drivetrain));

    // right8.onTrue(DriveCommands.lockToNearestShootingPosition(drivetrain));

    /**
     * INTAKE COMMANDS
     */
    // left1.whileTrue(new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake));
    // Trigger fuelInRange = new Trigger(() -> drivetrain.drivetrainSeesFuel() && DriverStation.isTeleop());
    
    // left1.and(right1.negate()).and(fuelInRange).whileTrue(DriveCommands.autoPickUp(leftJoystickX, leftJoystickY, drivetrain));
    // left1.whileTrue(RobotCommands.intake(intake, articulator));
    left1.and(right1.negate()).whileTrue(new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake));
    left1.whileTrue(new RunCommand(() -> articulator.setPosition(ArticulatorPosition.OUT), articulator));

    right2.onTrue(new RunCommand(() -> articulator.setPosition(ArticulatorPosition.IN), articulator));

    /**
     * SHOOTING/PASSING COMMANDS
     */

    right1.whileTrue(DriveCommands.shootOnTheMove(leftJoystickY, leftJoystickX, drivetrain));
    right1.whileTrue(new RunCommand(() -> shooter.setVelocityRPM(shooter.getStaticShootingRPM(true)), shooter));
    right1.whileTrue(new RunCommand(() -> hood.setPosition(hood.getHoodToFirePosition(true)), hood));
    
    // Trigger cannotFire = new Trigger(() -> shooter.isAtShootingRPM() && hood.isInPosition() && drivetrain.isPointingAtVector() && DriverStation.isTeleop());
    Trigger canPass = new Trigger(() -> shooter.getShootingTrigger() && hood.isInPosition() && drivetrain.isPointingAtVector() && !drivetrain.isDrivetrainInAllianceZone() && DriverStation.isTeleop());
    Trigger canShoot = new Trigger(() -> shooter.getShootingTrigger() && hood.isInPosition() && drivetrain.isPointingAtVector() && drivetrain.isDrivetrainInAllianceZone() && drivetrain.getShift().isOurHubActive() &&  DriverStation.isTeleop());

    Trigger firing = canShoot.or(canPass);

    right1.and(canShoot).onTrue(
      RobotCommands.agitateIntake(articulator)
    );

    right1.and(firing).whileTrue(
      new ParallelCommandGroup(
        RobotCommands.feedShooter(indexer, kicker)
    ));

    gamepadRT.and(canShoot).whileTrue(new ParallelCommandGroup(
      new RunCommand(() -> indexer.setSpeed(IndexerSpeed.INDEX), indexer),
      new RunCommand(() -> kicker.setSpeed(KickerSpeed.INDEX), kicker)
    ));

    gamepadLT.onTrue(new ParallelCommandGroup(
      new RunCommand(() -> indexer.setSpeed(IndexerSpeed.STOP), indexer),
      new RunCommand(() -> kicker.setSpeed(KickerSpeed.STOP), kicker)
    ));

    // remove lockToHub if nearestShootingLock works as it will be a double drivetrain requirement
    // right1.whileTrue(
    //   new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //       // new RunCommand(() -> hood.setPosition(0.0), hood),
    //       DriveCommands.lockToNearestDrivingAction(leftJoystickY, leftJoystickX, drivetrain),
    //       new RunCommand(() -> shooter.setVelocityRPM(2600.0), shooter)
    //     ).until(() -> /*hood.isInPosition() && */shooter.isShooterAtManualShotRPM()),
    //     new ParallelCommandGroup(
    //       // new RunCommand(() -> hood.setPosition(0.0), hood),
    //       DriveCommands.lockToNearestDrivingAction(leftJoystickY, leftJoystickX, drivetrain),
    //       new RunCommand(() -> shooter.setVelocityRPM(2600.0), shooter),
    //       new RunCommand(() -> indexer.setSpeed(IndexerSpeed.INDEX), indexer),
    //       new RunCommand(() -> kicker.setSpeed(KickerSpeed.INDEX), kicker)
    //     )
    //   )
    // ); 
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
