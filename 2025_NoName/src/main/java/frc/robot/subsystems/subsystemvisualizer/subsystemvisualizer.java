// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subsystemvisualizer;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaegroundintake.rollers.IntakeRollers;
import frc.robot.subsystems.algaegroundintake.wrist.IntakeWrist;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endefector.rollers.Rollers;
import frc.robot.subsystems.endefector.wrist.Wrist;
import frc.robot.subsystems.leds.LEDs;

public class SubsystemVisualizer extends SubsystemBase {

  private Mechanism2d mech = new Mechanism2d(60, 60);
  private MechanismRoot2d root = mech.getRoot("Root", 30, 5);

  //climb ligaments
  private MechanismLigament2d climb = 
      root.append(new MechanismLigament2d("Elevator", ElevatorConstants.heights[4], 90, 6, new Color8Bit(Color.kBlue)));

  // elevator ligaments
  private MechanismLigament2d elevator = 
      root.append(new MechanismLigament2d("Elevator", ElevatorConstants.heights[4], 90, 6, new Color8Bit(Color.kBlue)));

  // endefector ligaments
  private MechanismLigament2d endefectorWrist = 
      elevator.append(new MechanismLigament2d("endefectorWrist", 6, 90, 6, new Color8Bit(Color.kPurple)));
  private MechanismLigament2d endefectorRollers = 
      endefectorWrist.append(new MechanismLigament2d("endefectorRollers", 3, 90, 6, new Color8Bit(Color.kGreen)));
  
  // algae ground intake ligaments
  private MechanismLigament2d algaeGroundIntakeWrist =
     elevator.append(new MechanismLigament2d("algaeGroundIntake", 6, 90, 6, new Color8Bit(Color.kRed)));
  private MechanismLigament2d algaeGroundIntakeRollers =
     algaeGroundIntakeWrist.append(new MechanismLigament2d("algaeGroundIntakeRollers", 6, 90, 6, new Color8Bit(Color.kAzure)));
  
  /** Creates a new subsystemvisualizer. */
  public SubsystemVisualizer() {
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("dongleMech2d", mech);
    System.out.println("wadwa");
  }

  public Command update(Elevator elevator, Climb climb, LEDs LEDs, IntakeWrist algaeWrist, IntakeRollers algaeRollers, Wrist endWrist, Rollers endRollers){
    return Commands.sequence(
        Commands.runOnce(() -> algaeGroundIntakeWrist.setAngle(12.0), this),
        Commands.runOnce(() -> endefectorWrist.setAngle(15.0), this)
    );
  }

  
}
