package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.reduxrobotics.sensors.canandcolor.DigoutChannel.Index;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Indexer;
import frc.robot.subsystems.hopper.Indexer.IndexerSpeed;
import frc.robot.subsystems.intake.Articulator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeSpeed;
import frc.robot.subsystems.outtake.Hood;
import frc.robot.subsystems.outtake.Kicker;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.outtake.Kicker.KickerSpeed;
import frc.robot.util.MercMath;
import frc.robot.subsystems.intake.Articulator.ArticulatorPosition;

public class RobotCommands {

    // public static Command intakeInCommand(Articulator articulator) {
    //      return new RunCommand(() -> articulator.setPosition(ArticulatorPosition.IN), articulator);
    // }

    // Considers that the default pos of the articulator is at SAFE POS
    public static Command intake(Intake intake, Articulator articulator) {
        return new ParallelCommandGroup(
            new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake),
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.OUT), articulator)
        );
    }

    // Considers that the default pos of the articulator is at SAFE POS
    public static Command stopIntake(Intake intake, Articulator articulator) {
        return new ParallelCommandGroup(
            new RunCommand(() -> intake.setSpeed(Intake.IntakeSpeed.STOP), intake),
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.SAFE), articulator)
        );
    }

    // public static Command intakeAuton(Intake intake, Articulator articulator, Hopper hopper) {
    //     return new SequentialCommandGroup(
    //         intake(intake, articulator).until(() -> hopper.hopperIsFull()),
    //         new ParallelCommandGroup(
    //             new RunCommand(() -> intake.setSpeed(Intake.IntakeSpeed.STOP), intake),
    //             new RunCommand(() -> articulator.setPosition(ArticulatorPosition.SAFE), articulator)
    //         )
    //     );
    // }

    public static Command agitateIntake(Articulator articulator) {
        return new SequentialCommandGroup(
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.SAFE), articulator).withTimeout(2.0),
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.IN), articulator).until(() -> articulator.isAtPosition(ArticulatorPosition.IN))
        );
    }

    public static Command runIndex(Indexer index) {
        return new RunCommand(() -> index.setSpeed(IndexerSpeed.INDEX), index);      
    }

    public static Command uptake(Kicker kicker) {
        return new RunCommand(() -> kicker.setSpeed(KickerSpeed.INDEX), kicker);
    }

    public static Command prepareHood(Hood hood) {
        return new RunCommand(() -> hood.setPosition(hood.getHoodToFirePosition()), hood);
    }

    // public static Command setShooterToHubRPM(Shooter shooter) {
    //     return new RunCommand(() -> shooter.setSpeed(shooter.getStaticShootingRPM(false)), shooter);
    // }

    public static Command feedShooter(Indexer indexer, Kicker kicker) {
        return new ParallelCommandGroup(
            runIndex(indexer),
            uptake(kicker)
            // agitateIntake(articulator) // Do this seperately? So that you can intake while firing?
        );
    }

    // public static Command setUpToShoot(Shooter shooter, Hood hood, Drivetrain drivetrain, Articulator articulator) {
    //     return new ParallelCommandGroup(
    //         prepareHood(hood),
    //         setShooterToHubRPM(shooter),
    //         new RunCommand(() -> articulator.setPosition(ArticulatorPosition.IN), articulator)
    //         // DriveCommands.shootOnTheMove(drivetrain)
    //     );
    // }

    // public static Command setUpToShootAuton(Shooter shooter, Hood hood, Articulator articulator) {
    //     return new ParallelCommandGroup(
    //         prepareHood(hood),
    //         setShooterToHubRPM(shooter),
    //         new RunCommand(() -> articulator.setPosition(ArticulatorPosition.IN), articulator)
    //     );
    // }

    //auton
    public static Command stopFire(Shooter shooter, Kicker kicker, Articulator articulator, Indexer indexer) {
        return new ParallelCommandGroup(
            new RunCommand(() -> kicker.setSpeed(KickerSpeed.STOP), kicker),
            new RunCommand(() -> indexer.setSpeed(IndexerSpeed.STOP), indexer),
            // new RunCommand(() -> shooter.stop()), //really shouldn't be doing this, shooter should be constantly running
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.OUT), articulator)
        );
    }


    // //calls shootOnTheMove so it'll lock driving in a certain direction
    // public static Command fireAuton(Shooter shooter, Kicker kicker, Hood hood, Indexer indexer, Articulator articulator, Hopper hopper) {
    //     BooleanSupplier canFire = 
    //         () -> shooter.isAtShootingRPM() && 
    //                 hood.isInPosition();
        
    //     return new SequentialCommandGroup(
    //         new SequentialCommandGroup(
    //             setUpToShootAuton(shooter, hood, articulator).until(canFire),
    //             new ParallelCommandGroup(
    //                 setUpToShootAuton(shooter, hood, articulator), // You want to keep setting up while firing
    //                 feedShooter(indexer, kicker)
    //                 // agitateIntake(articulator)
    //             ).withTimeout(3),//always shoot for 3 seconds, TODO: adjust later as needed  Why??
    //             stopFire(shooter, kicker, articulator, indexer)
    //         )
    //     );
    // }

    // calls shootOnTheMove so it'll lock driving in a certain direction
    // we don't use this actually
    // public static Command fire(Shooter shooter, Kicker kicker, Hood hood, Indexer indexer, Drivetrain drivetrain, Articulator articulator) {
    //     BooleanSupplier canFire = 
    //         () -> shooter.isAtShootingRPM() && 
    //                 hood.isInPosition() &&
    //                 drivetrain.isPointingAtVector();
        
    //     return new SequentialCommandGroup(
    //         setUpToShoot(shooter, hood, drivetrain, articulator).until(canFire),
    //         new ParallelCommandGroup(
    //             setUpToShoot(shooter, hood, drivetrain, articulator), // You want to keep setting up while firing
    //             feedShooter(indexer, kicker)
    //         )
    //     );
    // }
}
 