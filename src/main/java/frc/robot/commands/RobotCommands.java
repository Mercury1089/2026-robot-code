package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Indexer;
import frc.robot.subsystems.intake.Articulator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Hood;
import frc.robot.subsystems.outtake.Kicker;
import frc.robot.subsystems.outtake.Shooter;
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

    public static Command shoot(Shooter shooter, Kicker kicker, Hood hood, Indexer indexer, Articulator articulator, Drivetrain drivetrain) {
        return new PrintCommand("Work in Progress");
    }
}
