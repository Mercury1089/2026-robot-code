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
import frc.robot.subsystems.outtake.Hood;
import frc.robot.subsystems.outtake.Kicker;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.subsystems.outtake.Kicker.KickerSpeed;
import frc.robot.util.MercMath;
import frc.robot.subsystems.intake.Articulator.ArticulatorPosition;

public class RobotCommands {

    public static Command intakeInCommand(Articulator articulator) {
         return new RunCommand(() -> articulator.setPosition(ArticulatorPosition.IN), articulator).until(() -> articulator.isAtPosition(ArticulatorPosition.IN));
    }

    // Considers that the default pos of the articulator is at SAFE POS
    public static Command intake(Intake intake, Articulator articulator) {
        return new ParallelCommandGroup(
            new RunCommand(() -> intake.setSpeed(Intake.IntakeSpeed.INTAKE), intake),
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.OUT), articulator).until(() -> articulator.isAtPosition(ArticulatorPosition.OUT))
        );
    }

    // Considers that the default pos of the articulator is at SAFE POS
    public static Command stopIntake(Intake intake, Articulator articulator) {
        return new ParallelCommandGroup(
            new RunCommand(() -> intake.setSpeed(Intake.IntakeSpeed.STOP), intake),
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.SAFE), articulator).until(() -> articulator.isAtPosition(ArticulatorPosition.OUT))
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
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.IN), articulator).until(() -> articulator.isAtPosition(ArticulatorPosition.IN)),
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.SAFE), articulator).until(() -> articulator.isAtPosition(ArticulatorPosition.SAFE))
        ).repeatedly();
    }

    public static Command runIndex(Indexer index) {
        return new RunCommand(() -> index.setSpeed(IndexerSpeed.INDEX), index);      
    }

    public static Command uptake(Kicker kicker) {
        return new RunCommand(() -> kicker.setSpeed(KickerSpeed.INDEX), kicker);
    }

    public static Command prepareHood(Hood hood) {
        return new RunCommand(() -> hood.setPosition(hood.getHoodToHubPosition()), hood);
    }

    public static Command setShooterToHubRPM(Shooter shooter) {
        return new RunCommand(() -> shooter.setSpeed(shooter.getStaticShootingRPM()), shooter);
    }

    public static Command feedShooter(Indexer indexer, Kicker kicker, Articulator articulator) {
        return new ParallelCommandGroup(
            runIndex(indexer),
            uptake(kicker),
            agitateIntake(articulator)
        );
    }

    public static Command setUpToShoot(Shooter shooter, Hood hood, Drivetrain drivetrain) {
        return new ParallelCommandGroup(
            prepareHood(hood),
            setShooterToHubRPM(shooter),
            DriveCommands.shootOnTheMove(drivetrain)
        );
    }

    //calls shootOnTheMove so it'll lock driving in a certain direction
    public static Command fire(Shooter shooter, Kicker kicker, Hood hood, Indexer indexer, Articulator articulator, Drivetrain drivetrain) {
        BooleanSupplier canFire = 
            () -> shooter.isAtShootingRPM() && 
                    hood.isInPosition() &&
                    drivetrain.isPointingAtHub();
        return new SequentialCommandGroup(
            setUpToShoot(shooter, hood, drivetrain).until(canFire),
            new ParallelCommandGroup(
                setUpToShoot(shooter, hood, drivetrain), // You want to keep setting up while firing
                feedShooter(indexer, kicker, articulator)
            )
        );
    }

    public static Command stopFire(Shooter shooter, Kicker kicker, Articulator articulator, Indexer indexer) {
        return new ParallelCommandGroup(
            new RunCommand(() -> kicker.setSpeed(KickerSpeed.STOP), kicker),
            new RunCommand(() -> indexer.setSpeed(IndexerSpeed.STOP), indexer),
            new RunCommand(() -> shooter.stop()), //should we be doing this?
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.OUT), articulator)
        );
    }

    // //calls shootOnTheMove so it'll lock driving in a certain direction
    // public static Command fireAuton(Shooter shooter, Kicker kicker, Hood hood, Indexer indexer, Articulator articulator, Drivetrain drivetrain, Hopper hopper) {
    //     return new SequentialCommandGroup(
    //         fire(shooter, kicker, hood, indexer, articulator, drivetrain).until(() -> hopper.hopperIsEmpty()),
    //         new ParallelCommandGroup(
    //             new RunCommand(() -> kicker.setSpeed(KickerSpeed.STOP), kicker),
    //             new RunCommand(() -> indexer.setSpeed(IndexerSpeed.STOP), indexer),
    //             new RunCommand(() -> articulator.setPosition(ArticulatorPosition.OUT), articulator)
    //         )
    //     );
    // }
}
 