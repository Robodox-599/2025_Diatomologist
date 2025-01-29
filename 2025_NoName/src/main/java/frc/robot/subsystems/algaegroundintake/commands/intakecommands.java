package frc.robot.subsystems.algaegroundintake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.algaegroundintake.rollers.Rollers;
import frc.robot.subsystems.algaegroundintake.wrist.Wrist;

public class intakecommands extends Command {
    public static Command intakeDeployAndIntake(Wrist wrist, Rollers rollers) {
        return Commands.parallel(
            extendCommand(wrist),
            
        )

    }
}
