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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaegroundintake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.algaegroundintake.intakewrist.IntakeWrist;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endefector.rollers.Rollers;
import frc.robot.subsystems.endefector.wrist.Wrist;
import frc.robot.subsystems.leds.LEDs;

public class SubsystemVisualizer extends SubsystemBase {

  private Mechanism2d mech = new Mechanism2d(60, 60);
  private MechanismRoot2d root = mech.getRoot("Root", 30, 5);

  private Elevator elev;
  private IntakeWrist intakeWrist;
  private IntakeRollers intakeRollers;
  private Climb climb;
  private Wrist endWrist;
  private Rollers endRollers;
  private LEDs LEDs;
    
  //climb ligaments
  private MechanismLigament2d climbVis = 
      root.append(new MechanismLigament2d("climb", ElevatorConstants.heights[4], 90, 6, new Color8Bit(Color.kBlue)));

  // elevator ligaments
  private MechanismLigament2d elevatorVis = 
      root.append(new MechanismLigament2d("Elevator", ElevatorConstants.heights[1], 90, 6, new Color8Bit(Color.kBlue)));

  // endefector ligaments
  private MechanismLigament2d endefectorWristVis = 
      elevatorVis.append(new MechanismLigament2d("endefectorWrist", 6, 90, 6, new Color8Bit(Color.kPurple)));
  private MechanismLigament2d endefectorRollersVis = 
      endefectorWristVis.append(new MechanismLigament2d("endefectorRollers", 3, 90, 6, new Color8Bit(Color.kGreen)));
  
  // algae ground intake ligaments
  private MechanismLigament2d algaeGroundIntakeWristVis =
     elevatorVis.append(new MechanismLigament2d("algaeGroundIntake", 6, 90, 6, new Color8Bit(Color.kRed)));
  private MechanismLigament2d algaeGroundIntakeRollersVis =
     algaeGroundIntakeWristVis.append(new MechanismLigament2d("algaeGroundIntakeRollers", 6, 90, 6, new Color8Bit(Color.kAzure)));
  
  public SubsystemVisualizer(Elevator elevator, Climb climb, LEDs LEDs, IntakeWrist algaeWrist, IntakeRollers algaeRollers, Wrist endWrist, Rollers endRollers) {
    //this.elev = elevator; 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elev.getIO().updateInputs();
    climb.getIO().updateInputs();
    intakeRollers.getIO().updateInputs();
    intakeWrist.getIO().updateInputs();
    endWrist.getIO().updateInputs();
    endRollers.getIO().updateInputs();
    LEDs.getIO().updateInputs();

    SmartDashboard.putData("dongleMech2d", mech);
    System.out.println("wadwa");
  }
  
  public void updateElevator(){
    switch (elev.getIO().getCurrentState()) {
      case L1:
          elevatorVis.setLength(elev.getIO().getPositionInches());
          System.out.println(elev.getIO().getCurrentState());
          break;
      case L2:
          elevatorVis.setLength(elev.getIO().getPositionInches());
          System.out.println(elev.getIO().getCurrentState());
          break;
      case L3:
          elevatorVis.setLength(elev.getIO().getPositionInches());
          System.out.println(elev.getIO().getCurrentState());
          break;
      case L4:
          elevatorVis.setLength(elev.getIO().getPositionInches());
          System.out.println(elev.getIO().getCurrentState());
          break;
      default:
          elevatorVis.setLength(elev.getIO().getPositionInches());
          System.out.println(elev.getIO().getCurrentState());
          break;
    }
  } 
  
  public void updateIntakeWrist(){
    switch (intakeWrist.getIO().getCurrentState()) {
      case DEPLOYED:
          algaeGroundIntakeWristVis.setAngle(intakeWrist.getIO().getCurrentPosition());
          algaeGroundIntakeWristVis.setColor(new Color8Bit(Color.kGreen));
          System.out.println(elev.getIO().getCurrentState());
          break;
      case NOTDEPLOYED:
          algaeGroundIntakeWristVis.setAngle(intakeWrist.getIO().getCurrentPosition());
          algaeGroundIntakeWristVis.setColor(new Color8Bit(Color.kRed));
          System.out.println(elev.getIO().getCurrentState());
          break;
      default:
          algaeGroundIntakeWristVis.setAngle(intakeWrist.getIO().getCurrentPosition());
          algaeGroundIntakeWristVis.setColor(new Color8Bit(Color.kBlue));
          System.out.println(elev.getIO().getCurrentState());
          break;
    }
  }

  public void updateIntakeRollers(){
    switch (intakeRollers.getIO().getCurrentState()) {
      case DEPLOYED:
          algaeGroundIntakeRollersVis.setAngle(intakeRollers.getIO().GetCurrentVolts());
          algaeGroundIntakeRollersVis.setColor(new Color8Bit(Color.kGreen));
          System.out.println(elev.getIO().getCurrentState());
          break;
      case NOTDEPLOYED:
          algaeGroundIntakeRollersVis.setAngle(intakeRollers.getIO().GetCurrentVolts());
          algaeGroundIntakeRollersVis.setColor(new Color8Bit(Color.kRed));
          System.out.println(elev.getIO().getCurrentState());
          break;
      case REVERSED:
          algaeGroundIntakeRollersVis.setAngle((intakeRollers.getIO().GetCurrentVolts()) * -1);
          algaeGroundIntakeRollersVis.setColor(new Color8Bit(Color.kPurple));
          System.out.println(elev.getIO().getCurrentState());
          break;
      case IDLE:
          algaeGroundIntakeRollersVis.setAngle(intakeRollers.getIO().GetCurrentVolts());
          algaeGroundIntakeRollersVis.setColor(new Color8Bit(Color.kBlue));
          System.out.println(elev.getIO().getCurrentState());
          break;
      default:
      algaeGroundIntakeRollersVis.setAngle(intakeRollers.getIO().GetCurrentVolts());
          algaeGroundIntakeRollersVis.setColor(new Color8Bit(Color.kYellow));
          System.out.println(elev.getIO().getCurrentState());
          break;
      }

  }

  public void updateClimb(){
    switch (climb.getIO().getCurrentState()) {
      case CLIMB:
          climbVis.setAngle(climb.getIO().GetCurrentVolts());
          algaeGroundIntakeRollersVis.setColor(new Color8Bit(Color.kGreen));
          System.out.println(climb.getIO().getCurrentState());
          break;
      case CLIMBREADY:
          climbVis.setAngle(climb.getIO().GetCurrentVolts());
          climbVis.setColor(new Color8Bit(Color.kYellow));
          System.out.println(climb.getIO().getCurrentState());
          break;
      default:
          climbVis.setAngle(climb.getIO().GetCurrentVolts());
          climbVis.setColor(new Color8Bit(Color.kBlue));
          System.out.println(climb.getIO().getCurrentState());
          break;
      }
  }

  public void updateEndefectorWrist(){
    switch (endWrist.getIO().getCurrentState()) {
      case SCORING:
          endefectorWristVis.setAngle(endWrist.getIO().getCurrentPosition());
          endefectorWristVis.setColor(new Color8Bit(Color.kRed));
          System.out.println(elev.getIO().getCurrentState());
          break;
      case OVERRIDE:
          endefectorWristVis.setAngle(intakeWrist.getIO().getCurrentPosition());
          endefectorWristVis.setColor(new Color8Bit(Color.kGreen));
          System.out.println(elev.getIO().getCurrentState());
          break;
      case GROUNDINTAKE:
          endefectorWristVis.setAngle(intakeWrist.getIO().getCurrentPosition());
          endefectorWristVis.setColor(new Color8Bit(Color.kRed));
          System.out.println(elev.getIO().getCurrentState());
          break;
      case STATIONINTAKE:
          endefectorWristVis.setAngle(intakeWrist.getIO().getCurrentPosition());
          endefectorWristVis.setColor(new Color8Bit(Color.kGreen));
          System.out.println(elev.getIO().getCurrentState());
          break;
      case CLIMB:
          endefectorWristVis.setAngle(intakeWrist.getIO().getCurrentPosition());
          endefectorWristVis.setColor(new Color8Bit(Color.kRed));
          System.out.println(elev.getIO().getCurrentState());
          break;
      default: // STOW
          endefectorWristVis.setAngle(intakeWrist.getIO().getCurrentPosition());
          endefectorWristVis.setColor(new Color8Bit(Color.kBlue));
          System.out.println(elev.getIO().getCurrentState());
          break;
    }
  }

  public void UpdateEndefectorRollers(){
    switch (endRollers.getIO().getCurrentState()) {
      case DEPLOYED:
          endefectorRollersVis.setAngle(endRollers.getIO().GetCurrentVolts());
          endefectorRollersVis.setColor(new Color8Bit(Color.kGreen));
          System.out.println(elev.getIO().getCurrentState());
          break;
      case NOTDEPLOYED:
          endefectorRollersVis.setAngle(endRollers.getIO().GetCurrentVolts());
          endefectorRollersVis.setColor(new Color8Bit(Color.kRed));
          System.out.println(elev.getIO().getCurrentState());
          break;
      case REVERSED:
          endefectorRollersVis.setAngle((endRollers.getIO().GetCurrentVolts()) * -1);
          endefectorRollersVis.setColor(new Color8Bit(Color.kPurple));
          System.out.println(elev.getIO().getCurrentState());
          break;
      case IDLE:
          endefectorRollersVis.setAngle(endRollers.getIO().GetCurrentVolts());
          endefectorRollersVis.setColor(new Color8Bit(Color.kBlue));
          System.out.println(elev.getIO().getCurrentState());
          break;
      default:
          endefectorRollersVis.setAngle(endRollers.getIO().GetCurrentVolts());
          endefectorRollersVis.setColor(new Color8Bit(Color.kYellow));
          System.out.println(elev.getIO().getCurrentState());
          break;
      }
  }

  public void updateLEDs(){
    switch (LEDs.getIO().getCurrentState()) {
      case YesNote:
          break;
      case NoNote:
          break;
      case CanShoot:
          break;
      default: // NoState
          break;
      }
    
  }
}
