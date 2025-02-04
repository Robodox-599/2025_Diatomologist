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
import frc.robot.subsystems.algaegroundintake.intakeWrist.IntakeWrist;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endefector.rollers.Rollers;
import frc.robot.subsystems.endefector.wrist.Wrist;
import frc.robot.subsystems.leds.LEDs;

public class SubsystemVisualizer extends SubsystemBase {

  private Mechanism2d mech = new Mechanism2d(60, 60);
  private MechanismRoot2d root = mech.getRoot("Root", 30, 5);

  private LEDs LEDs;
  private IntakeRollers algaeRollers;
  private IntakeWrist algaeWrist;
  private Elevator elevator;
  private Rollers endefectorRollers;
  private Wrist endefectorWrist;
  private Climb climb;

  // climb ligaments
  private MechanismLigament2d climbVis =
      root.append(
          new MechanismLigament2d(
              "climb", ElevatorConstants.heights[4], 90, 6, new Color8Bit(Color.kBlue)));

  // elevator ligaments
  private MechanismLigament2d elevatorVis =
      root.append(
          new MechanismLigament2d(
              "Elevator", ElevatorConstants.heights[1], 90, 6, new Color8Bit(Color.kBlue)));

  // endefector ligaments
  private MechanismLigament2d endefectorWristVis =
      elevatorVis.append(
          new MechanismLigament2d("endefectorWrist", 6, 90, 6, new Color8Bit(Color.kPurple)));
  private MechanismLigament2d endefectorRollersVis =
      endefectorWristVis.append(
          new MechanismLigament2d("endefectorRollers", 3, 90, 6, new Color8Bit(Color.kGreen)));

  // algae ground intake ligaments
  private MechanismLigament2d algaeGroundIntakeWristVis =
      elevatorVis.append(
          new MechanismLigament2d("algaeGroundIntake", 6, 90, 6, new Color8Bit(Color.kRed)));
  private MechanismLigament2d algaeGroundIntakeRollersVis =
      algaeGroundIntakeWristVis.append(
          new MechanismLigament2d(
              "algaeGroundIntakeRollers", 6, 90, 6, new Color8Bit(Color.kAzure)));

  public SubsystemVisualizer(
      Elevator elevator,
      Climb climb,
      LEDs LEDs,
      IntakeWrist algaeWrist,
      IntakeRollers algaeRollers,
      Wrist endWrist,
      Rollers endRollers) {
    this.elevator = elevator;
    this.climb = climb;
    this.LEDs = LEDs;
    this.algaeWrist = algaeWrist;
    this.algaeRollers = algaeRollers;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateElevator();
    updateIntakeWrist();
    updateIntakeRollers();
    updateClimb();
    updateEndefectorWrist();
    updateEndefectorRollers();
    SmartDashboard.putData("dongleMech2d", mech);
  }

  public void updateElevator() {
    switch (elevator.getIO().getCurrentState()) {
      case L1:
        elevatorVis.setLength(elevator.getIO().getPositionInches());
        break;
      case L2:
        elevatorVis.setLength(elevator.getIO().getPositionInches());
        break;
      case L3:
        elevatorVis.setLength(elevator.getIO().getPositionInches());
        break;
      case L4:
        elevatorVis.setLength(elevator.getIO().getPositionInches());
        break;
      default:
        elevatorVis.setLength(elevator.getIO().getPositionInches());
        break;
    }
  }

  public void updateIntakeWrist() {
    switch (algaeWrist.getIO().getCurrentState()) {
      case DEPLOYED:
        algaeGroundIntakeWristVis.setAngle(algaeWrist.getIO().getCurrentPosition());
        algaeGroundIntakeWristVis.setColor(new Color8Bit(Color.kGreen));
        break;
      case NOTDEPLOYED:
        algaeGroundIntakeWristVis.setAngle(algaeWrist.getIO().getCurrentPosition());
        algaeGroundIntakeWristVis.setColor(new Color8Bit(Color.kRed));
        break;
      default:
        algaeGroundIntakeWristVis.setAngle(algaeWrist.getIO().getCurrentPosition());
        algaeGroundIntakeWristVis.setColor(new Color8Bit(Color.kBlue));
        break;
    }
  }

  public void updateIntakeRollers() {
    switch (algaeRollers.getIO().getCurrentState()) {
      case DEPLOYED:
        algaeGroundIntakeRollersVis.setAngle(algaeRollers.getIO().GetCurrentVolts());
        algaeGroundIntakeRollersVis.setColor(new Color8Bit(Color.kGreen));
        break;
      case NOTDEPLOYED:
        algaeGroundIntakeRollersVis.setAngle(algaeRollers.getIO().GetCurrentVolts());
        algaeGroundIntakeRollersVis.setColor(new Color8Bit(Color.kRed));
        break;
      case REVERSED:
        algaeGroundIntakeRollersVis.setAngle((algaeRollers.getIO().GetCurrentVolts()) * -1);
        algaeGroundIntakeRollersVis.setColor(new Color8Bit(Color.kPurple));
        break;
      case IDLE:
        algaeGroundIntakeRollersVis.setAngle(algaeRollers.getIO().GetCurrentVolts());
        algaeGroundIntakeRollersVis.setColor(new Color8Bit(Color.kBlue));
        break;
      default:
        algaeGroundIntakeRollersVis.setAngle(algaeRollers.getIO().GetCurrentVolts());
        algaeGroundIntakeRollersVis.setColor(new Color8Bit(Color.kYellow));
        break;
    }
  }

  public void updateClimb() {
    switch (climb.getIO().getCurrentState()) {
      case CLIMB:
        climbVis.setAngle(climb.getIO().GetCurrentVolts());
        algaeGroundIntakeRollersVis.setColor(new Color8Bit(Color.kGreen));
        break;
      case CLIMBREADY:
        climbVis.setAngle(climb.getIO().GetCurrentVolts());
        climbVis.setColor(new Color8Bit(Color.kYellow));
        break;
      default:
        climbVis.setAngle(climb.getIO().GetCurrentVolts());
        climbVis.setColor(new Color8Bit(Color.kBlue));
        break;
    }
  }

  public void updateEndefectorWrist() {
    switch (endefectorWrist.getIO().getCurrentState()) {
      case SCORING:
        endefectorWristVis.setAngle(endefectorWrist.getIO().getCurrentPosition());
        endefectorWristVis.setColor(new Color8Bit(Color.kRed));
        break;
      case OVERRIDE:
        endefectorWristVis.setAngle(endefectorWrist.getIO().getCurrentPosition());
        endefectorWristVis.setColor(new Color8Bit(Color.kGreen));
        break;
      case GROUNDINTAKE:
        endefectorWristVis.setAngle(endefectorWrist.getIO().getCurrentPosition());
        endefectorWristVis.setColor(new Color8Bit(Color.kRed));
        break;
      case STATIONINTAKE:
        endefectorWristVis.setAngle(endefectorWrist.getIO().getCurrentPosition());
        endefectorWristVis.setColor(new Color8Bit(Color.kGreen));
        break;
      case CLIMB:
        endefectorWristVis.setAngle(endefectorWrist.getIO().getCurrentPosition());
        endefectorWristVis.setColor(new Color8Bit(Color.kRed));
        break;
      default: // STOW
        endefectorWristVis.setAngle(endefectorWrist.getIO().getCurrentPosition());
        endefectorWristVis.setColor(new Color8Bit(Color.kBlue));
        break;
    }
  }

  public void updateEndefectorRollers() {
    switch (endefectorRollers.getIO().getCurrentState()) {
      case DEPLOYED:
        endefectorRollersVis.setAngle(endefectorRollers.getIO().GetCurrentVolts());
        endefectorRollersVis.setColor(new Color8Bit(Color.kGreen));
        break;
      case NOTDEPLOYED:
        endefectorRollersVis.setAngle(endefectorRollers.getIO().GetCurrentVolts());
        endefectorRollersVis.setColor(new Color8Bit(Color.kRed));
        break;
      case REVERSED:
        endefectorRollersVis.setAngle((endefectorRollers.getIO().GetCurrentVolts()) * -1);
        endefectorRollersVis.setColor(new Color8Bit(Color.kPurple));
        break;
      case IDLE:
        endefectorRollersVis.setAngle(endefectorRollers.getIO().GetCurrentVolts());
        endefectorRollersVis.setColor(new Color8Bit(Color.kBlue));
        break;
      default:
        endefectorRollersVis.setAngle(endefectorRollers.getIO().GetCurrentVolts());
        endefectorRollersVis.setColor(new Color8Bit(Color.kYellow));
        break;
    }
  }
}
