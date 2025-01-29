package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.Constants.*;

public class RobotContainer {
  private Elevator elevator;

  private final CommandXboxController xboxController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        elevator = new Elevator(new ElevatorIOTalonFX());
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        elevator = new Elevator(new ElevatorIOSim());
        System.out.println("In Sim");
        break;
    }
    configureBindings();
  }

  private void configureBindings() {
    // Xbox Controller Bindings
    xboxController.a().whileTrue(Commands.sequence(elevator.moveToState(ElevatorConstants.ElevatorStates.L1)));
    xboxController.b().whileTrue(Commands.sequence(elevator.moveToState(ElevatorConstants.ElevatorStates.L2)));
    xboxController.y().whileTrue(Commands.sequence(elevator.moveToState(ElevatorConstants.ElevatorStates.L3)));
    xboxController.x().whileTrue(Commands.sequence(elevator.moveToState(ElevatorConstants.ElevatorStates.L4)));
}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}