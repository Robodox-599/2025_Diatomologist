// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subsystemvisualizer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.leds.LEDs;

public class SubsystemVisualizer extends SubsystemBase {

  Climb climb;
  Elevator elevator;
  Rollers endefectorRollers;
  Wrist endefectorWrist;
  LEDs lightEmittingDiode;

  private double currentRollersVisAngle = 0; // Persistent angle to track rotation

  Mechanism2d mech = new Mechanism2d(60, 60);
  MechanismRoot2d root = mech.getRoot("root", 30, 0);
  
  MechanismLigament2d elevatorVis =
  root.append(new MechanismLigament2d("elevator", 10, 90, 8, new Color8Bit(Color.kRed)));
  
  MechanismLigament2d climbVis =
  root.append(new MechanismLigament2d("climb", 9, 45, 6, new Color8Bit(Color.kGreen)));
  
  MechanismLigament2d endfectorWristVis =
  elevatorVis.append(
      new MechanismLigament2d("endefectorWristVis", 6, -45, 4, new Color8Bit(Color.kPurple)));
      
  MechanismLigament2d endfectorRollersVis =
  endfectorWristVis.append(
      new MechanismLigament2d("endefectorRollersVis", 3, -45, 4, new Color8Bit(Color.kYellow)));
          
  public SubsystemVisualizer(Elevator elevator, Climb climb, Wrist wrist, Rollers rollers) {
    this.elevator = elevator;
    this.climb = climb;
    this.endefectorWrist = wrist;
    this.endefectorRollers = rollers;
  }

  @Override
  public void periodic() {
    updateElevator();
    updateClimb();
    updateWrist();
    updateRollers();

    SmartDashboard.putData("DongleMechanism2D", mech);
  }

  public void updateElevator() {
    elevatorVis.setLength(Units.inchesToMeters(elevator.getIO().getPositionInches()) * 25);
  }

  public void updateClimb() {
    climbVis.setAngle(Units.inchesToMeters(climb.getIO().getPositionInches()) * 25);
  }

  public void updateWrist() {
    endfectorWristVis.setAngle(
        Units.inchesToMeters(endefectorWrist.getIO().getCurrentPosition()) * 25);
  }

  public void updateRollers() {
    currentRollersVisAngle = (currentRollersVisAngle + endefectorRollers.getIO().getVelocity()) % 360;
    endfectorRollersVis.setAngle(currentRollersVisAngle);
  }
}
