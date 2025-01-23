// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;

public class RobotContainer {
  private Climb climb;

  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer(){
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        climb = new Climb(new ClimbIOTalonFX());

        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        climb = new Climb(new ClimbIOSim());
        break;
    }
      DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));

    configureBindings();
  }

  private void configureBindings() {
    //controller.a().onTrue(climb.move(2));

    controller.a().whileTrue(climb.move(2));

    //controller.x().whileTrue(climb.move(2).onFalse(climb.stop());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
