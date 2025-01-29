// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.endefector.rollers.Rollers;
import frc.robot.endefector.rollers.RollersIOSim;
import frc.robot.endefector.rollers.RollersIOTalonFX;

import frc.robot.endefector.wrist.Wrist;
import frc.robot.endefector.wrist.WristIOSim;
import frc.robot.endefector.wrist.WristIOTalonFX;

public class RobotContainer {
  private Rollers rollers;
  private Wrist wrist;

  private final CommandXboxController controller =
  new CommandXboxController(Constants.driverControllerPort);
// boom
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        rollers = new Rollers(new RollersIOTalonFX());
        wrist = new Wrist(new WristIOTalonFX());

        break;
      case SIM:
        rollers = new Rollers(new RollersIOSim());
        wrist = new Wrist(new WristIOSim());
       
        break;
    }

      DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));

    configureBindings();
  }
  

  private void configureBindings() {
    // controller.x().onFalse(wrist.stop());
    // controller.a().onFalse(rollers.stop());

    controller.x().whileTrue(wrist.goToPose(2));
    controller.a().whileTrue(rollers.setVelocity(2));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}