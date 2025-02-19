// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subsystemvisualizer;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.leds.LEDs;

public class SubsystemVisualizer extends SubsystemBase {

  Climb climb;
  Elevator elevator;
  Rollers endefectorRollers;
  Wrist endefectorWrist;
  LEDs lightEmittingDiode;
  
  Mechanism2d mech = new Mechanism2d(3, 3);
  MechanismRoot2d root = mech.getRoot("climber", 2, 0);
  
  MechanismLigament2d elevatorVis = 
    root.append(new MechanismLigament2d("elevator", ElevatorConstants.elevatorLowerLimit, 90));

  MechanismLigament2d climbVis = 
    root.append(new MechanismLigament2d("elevator", 3, 90, 4, new Color8Bit(Color.kPurple)));
  
  MechanismLigament2d endfectorWristVis = 
    elevatorVis.append(new MechanismLigament2d("endefectorWristVis", 3, 90, 4, new Color8Bit(Color.kPurple)));

  MechanismLigament2d endfectorRollersVis = 
    endfectorWristVis.append(new MechanismLigament2d("endefectorRollersVis", 3, 90, 4, new Color8Bit(Color.kPurple)));

  public SubsystemVisualizer() {

  }

  @Override
  public void periodic() {
    
  }
}
