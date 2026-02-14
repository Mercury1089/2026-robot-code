package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Articulator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Articulator.ArticulatorPosition;

public class RobotCommands {

    public static Command intakeInCommand(Hopper hopper, Articulator articulator) {
         return new SequentialCommandGroup(
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.IN), articulator).until(() -> articulator.isAtPosition(ArticulatorPosition.IN)),
            new RunCommand(null, null));
        
    }



}
