package frc.robot.commands;

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
        return new RunCommand(() -> hood.isInPosition(), hood);
    }

    public static Command runShooter(Shooter shooter) {
        return new RunCommand(() -> shooter.setSpeed(shooter.getShootingRPM()), shooter);
    }

    public static Command shoot(Shooter shooter, Kicker kicker, Hood hood, Indexer indexer, Articulator articulator, Drivetrain drivetrain) {
        return new ParallelCommandGroup(
            DriveCommands.shootOnTheMove(drivetrain),
            agitateIntake(articulator),
            runIndex(indexer),
            uptake(kicker),
            prepareHood(hood),
            runShooter(shooter).onlyIf(() -> shooter.getVelocityRPM() == shooter.getShootingRPM())
        );
    }
}
 