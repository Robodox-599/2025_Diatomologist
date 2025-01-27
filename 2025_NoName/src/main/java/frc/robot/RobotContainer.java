// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Thread.State;

import com.ctre.phoenix6.hardware.TalonFX;

import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
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

  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer(){
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        elevator = new Elevator(new ElevatorIOTalonFX());

        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        elevator = new Elevator(new ElevatorIOSim());
        break;
    }
      DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));

    configureBindings();
  }

  private void configureBindings() {
    // controller.a().onTrue(Commands.sequence(elevator.moveToState(ElevatorConstants.ElevatorStates.L4)));
    controller.a().whileTrue(elevator.move(2));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
