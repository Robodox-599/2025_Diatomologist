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
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsIOReal;
import frc.robot.subsystems.leds.LEDsIOSim;
import frc.robot.subsystems.leds.LEDsConstants.*;

public class RobotContainer {
  private LEDs lightEmittingDiode;
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer(){
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        lightEmittingDiode = new LEDs(new LEDsIOReal());
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        lightEmittingDiode = new LEDs(new LEDsIOSim());
        break;
    }
      DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));

    configureBindings();
  }

  private void configureBindings() {
    //controller.a().onTrue(climb.move(2));

    controller.a().whileTrue(lightEmittingDiode.runStationIntake());

    controller.b().whileTrue(lightEmittingDiode.runAlgaeIntake());

    controller.x().whileTrue(lightEmittingDiode.runNoState());

    controller.y().whileTrue(lightEmittingDiode.runScored());

    controller.rightBumper().whileTrue(lightEmittingDiode.runClimb());

    controller.leftBumper().whileTrue(lightEmittingDiode.runAutoAlign());

    controller.rightTrigger().whileTrue(lightEmittingDiode.runReadyToScore());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
