package frc.robot.subsystems.algaegroundintake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.algaegroundintake.rollers.intakeRollers;
import frc.robot.subsystems.algaegroundintake.wrist.intakeWrist;

public class intakecommands extends Command {
    public static Command intakeDeployAndIntake(intakeWrist wrist, intakeRollers rollers) {
        return Commands.parallel(
            extendCommand(wrist),
            rollers.setVoltage(2));


    }
}
