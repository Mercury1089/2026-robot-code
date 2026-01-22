package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    /**
    * @param : Supplier (xSpeed, ySpeed), 
    * @return : Returns a RunCommand telling the drivetrain to drive and calculates heading degrees required to target
    */
    public static Command targetDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> headingSupplier, Drivetrain drivetrain) {
        return new RunCommand(
                () -> drivetrain.drive(
                  -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), headingSupplier.get()),
                  true)
              , drivetrain);
    }

    /**
    * @param: Drivetrain, Pose2d Supplier 
    * @return: Outputs a Run Command 
    */
    public static Command driveToPose(Drivetrain drivetrain, Supplier<Pose2d> desiredPose) {
        return new RunCommand(
            () -> drivetrain.drive(
              drivetrain.getXController().calculate(drivetrain.getPose().getX(), desiredPose.get().getX()),
              drivetrain.getYController().calculate(drivetrain.getPose().getY(), desiredPose.get().getY()),
              drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), desiredPose.get().getRotation().getDegrees()),
              true, false,
              () -> drivetrain.getPose().getRotation())
          , drivetrain);
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
   
    public static Command driveStraightAtAngle(Supplier<Double> headingSupplier, double speed, Drivetrain drivetrain) {
        double magnitude = Math.hypot(drivetrain.getXSpeeds(), drivetrain.getYSpeeds());
        double xSpeedCapped = (magnitude > 1e-6) ? speed * (drivetrain.getXSpeeds() / magnitude) : 0.0;
        double ySpeedCapped = (magnitude > 1e-6) ? speed * (drivetrain.getYSpeeds() / magnitude) : 0.0;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            xSpeedCapped,
            ySpeedCapped,
            0.0 // TODO: Give Radial Velocity
        );
        return new RunCommand(
            () -> drivetrain.drive(chassisSpeeds),
            drivetrain
        );
    }

    public static Command safelyDriveOverBump(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Drivetrain drivetrain) {
        // These headings might need to be adjusted based on the ratio of the length and width of the robot
        // to ensure that the robot's wheel always hit the bump first.
        Supplier<Double> nearestSafeHeading = () -> drivetrain.getSafeBumpingAngle();

        return new RunCommand(
                () -> drivetrain.drive(
                  -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), nearestSafeHeading.get()),
                  true)
              , drivetrain);
    }

    public static Command lockToHub(Supplier<Double> xSupplier, Supplier<Double> ySupplier, Drivetrain drivetrain) {
        Supplier<Double> headingSupplier = () -> TargetUtils.getTargetHeadingToPoint(drivetrain.getPose(), KnownLocations.getKnownLocations().HUB.getTranslation()).getDegrees();
        return targetDrive(xSupplier, ySupplier, headingSupplier, drivetrain);
    }

}