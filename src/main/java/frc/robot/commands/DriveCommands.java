package frc.robot.commands;

import java.sql.Driver;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SWERVE;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.KnownLocations;
import frc.robot.util.MercMath;
import frc.robot.util.PathUtils;
import frc.robot.util.TargetUtils;

public class DriveCommands {
    /**
    * @param  : Drivetrain, Suppliers
    * @return  :Drives based on the Joystick values given, as a Run Command
    */
    public static Command joyStickDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> angularSpeedSupplier, Drivetrain drivetrain) {
        return new RunCommand(
            () -> drivetrain.drive(
              -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.squareInput(MathUtil.applyDeadband(angularSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)))
          , drivetrain);
    }
    /** 
    * @param : Suppliers (xSpeed, ySpeed, angularSpeed), Drivetrain, Boolean (Field Relative)
    * @return : RunCommand
    */
    public static Command joyStickDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> angularSpeedSupplier, boolean fieldRelative, Drivetrain drivetrain) {
        return new RunCommand(
            () -> drivetrain.drive(
              -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.squareInput(MathUtil.applyDeadband(angularSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              fieldRelative)
          , drivetrain);
    } 
    // /**
    // * @param : Supplier (xSpeed, ySpeed, heading), Drivetrain
    // * @return : Returns PID Command (Pose, Rotation, and Heading Degrees)
    // */
    // public static Command targetDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, DoubleSupplier headingSupplier, Drivetrain drivetrain) {
    //     return new PIDCommand(
    //         drivetrain.getRotationalController(),
    //         () -> drivetrain.getPose().getRotation().getDegrees(),
    //         headingSupplier,
    //         (angularSpeed) -> drivetrain.drive(
    //           -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
    //           -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
    //         angularSpeed),
    //         drivetrain);
    // }
    //TODO: Refactor with a Pose2d Parameter
    /**
    * @param : Supplier (xSpeed, ySpeed), 
    * @return : Returns a RunCommand telling the drivetrain to drive and calculates heading degrees required to target reef
    */
    // public static Command targetDriveToReef(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Drivetrain drivetrain) {
    //     Supplier<Double> heading = () -> ReefscapeUtils.getTargetHeadingToReef(drivetrain.getPose());
    //     return new RunCommand(
    //             () -> drivetrain.drive(
    //               -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
    //               -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
    //               drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), heading.get()),
    //               true)
    //           , drivetrain);
    // }

    /**
    * @param : Drivetrain, Pose2d Supplier 
    * @return : Outputs a Run Command 
    */
    public static Command goToPose(Drivetrain drivetrain, Supplier<Pose2d> desiredPose) {
        return new RunCommand(
            () -> drivetrain.drive(
              drivetrain.getXController().calculate(drivetrain.getPose().getX(), desiredPose.get().getX()),
              drivetrain.getYController().calculate(drivetrain.getPose().getY(), desiredPose.get().getY()),
              drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), desiredPose.get().getRotation().getDegrees()),
              true, false,
              () -> drivetrain.getPose().getRotation())
          , drivetrain);//.until(() -> drivetrain.isAtPose(desiredPose.get()))
          //this seems like a bad idea to comment this out
    }
    
    /**
    * @param : Drivetrain
    * Sensor Input
    * @return : Outputs a Run Command and calculates if the robot is too far left or right it will adjust itself
    * SELECT BRANCH AND ZONE BEFORE USING
    */
    // public static Command alignwithSensors(Drivetrain drivetrain, Supplier<RobotZone> zone, Supplier<BranchSide> side) {

    //     Supplier<DistanceSensors> proximitySensor = () -> side.get() == BranchSide.LEFT ? drivetrain.getLeftSensors() : drivetrain.getRightSensors();

    //     // proximitySensor = () -> zone.get() == RobotZone.BARGE || zone.get() == RobotZone.BARGE_LEFT || zone.get() == RobotZone.BARGE_RIGHT ?
    //     //                         side.get() == BranchSide.LEFT ? drivetrain.getRightSensors() : drivetrain.getLeftSensors() :
    //     //                         side.get() == BranchSide.LEFT ? drivetrain.getLeftSensors() : drivetrain.getRightSensors();

    //     // Positive Y moves right, negative Y moves left
    //     Supplier<Double> yDirection = () -> proximitySensor.get().isTooFarLeft(zone, side) ? -1.0 : 1.0;

    //     return goCloserToReef(drivetrain, zone, side).andThen(new RunCommand(
    //         () -> drivetrain.drive(
    //           0.0, 
    //           yDirection.get() * 0.05,
    //           0.0,
    //           false)
    //     ).until(() -> proximitySensor.get().isAtReefSide())
    //         .andThen(new RunCommand(() -> drivetrain.drive(0.0,0.0,0.0))).until(() -> drivetrain.isNotMoving()));
    // }
    /**
    * SELECT SIDE AND CORAL STATION BEFORE USING
    * @param : Drivetrain 
    * @return : Returns a Sequential Command Group, and starts goToPose command to go to the prefered Station
    */
    public static Command goToCoralStation(Drivetrain drivetrain, Pose2d station) {
        return new SequentialCommandGroup(
            PathUtils.getPathToPose(station, () -> 0.5),
            goToPose(drivetrain, () -> station).until(() -> drivetrain.isAtPose(station)));
    }

    /**
     * Construct a command that will follow a path provided when the command initializes.
     * @param pathSupplier Supplier that provides the path to follow.
     * @param drivetrain Drivetrain subsystem that will follow the path
     * @return The Druvetrain path follows form path supplier
     */
    public static Command followPath(Supplier<PathPlannerPath> pathSupplier, Drivetrain drivetrain) {
        return drivetrain.defer(() -> AutoBuilder.followPath(pathSupplier.get()));
    }
   


}
