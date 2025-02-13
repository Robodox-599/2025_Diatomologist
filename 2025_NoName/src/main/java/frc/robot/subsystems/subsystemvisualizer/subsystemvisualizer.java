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
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorwrist.*;

public class subsystemvisualizer extends SubsystemBase {

  private Mechanism2d mech = new Mechanism2d(60, 60);
  private MechanismRoot2d root = mech.getRoot("Root", 30, 5);

  private Elevator elevator;
  private Rollers endefectorRollers;
  private Wrist endefectorWrist;
  private Climb climb;

  // climb ligaments
  private MechanismLigament2d climbVis =
      root.append(
          new MechanismLigament2d(
              "climb", ClimbConstants.setpoint[1], 90, 6, new Color8Bit(Color.kSeashell)));

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
  // private MechanismLigament2d algaeGroundIntakeWristVis =
  //     elevatorVis.append(
  //         new MechanismLigament2d("algaeGroundIntake", 6, 90, 6, new Color8Bit(Color.kRed)));
  // private MechanismLigament2d algaeGroundIntakeRollersVis =
  //     algaeGroundIntakeWristVis.append(
  //         new MechanismLigament2d(
  //             "algaeGroundIntakeRollers", 6, 90, 6, new Color8Bit(Color.kAzure)));

  public subsystemvisualizer(Elevator elevator, Climb climb, Wrist endWrist, Rollers endRollers) {
    this.elevator = elevator;
    this.climb = climb;
    this.endefectorWrist = endWrist;
    this.endefectorRollers = endRollers;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateElevator();
    updateClimb();
    updateEndefectorWrist();
    updateEndefectorRollers();
    SmartDashboard.putData("dongleMech2d", mech);
  }

  public void updateElevator() {
    switch (elevator.getIO().getState()) {
      case L1:
        elevatorVis.setLength(Units.inchesToMeters(elevator.getIO().getPositionInches()) * 3);
        break;
      case L2:
        elevatorVis.setLength(Units.inchesToMeters(elevator.getIO().getPositionInches()) * 3);
        break;
      case L3:
        elevatorVis.setLength(Units.inchesToMeters(elevator.getIO().getPositionInches()) * 3);
        break;
      case L4:
        elevatorVis.setLength(Units.inchesToMeters(elevator.getIO().getPositionInches()) * 3);
        break;
      case INTAKE:
        elevatorVis.setLength(Units.inchesToMeters(elevator.getIO().getPositionInches()) * 3);
        break;
      case GROUNDINTAKE:
        elevatorVis.setLength(Units.inchesToMeters(elevator.getIO().getPositionInches()) * 3);
        break;
      case ALGAE_L2:
        elevatorVis.setLength(Units.inchesToMeters(elevator.getIO().getPositionInches()) * 3);
        break;
      case ALGAE_L3:
        elevatorVis.setLength(Units.inchesToMeters(elevator.getIO().getPositionInches()) * 3);
        break;
      default: // stow
        elevatorVis.setLength(Units.inchesToMeters(elevator.getIO().getPositionInches()) * 3);
        break;
    }
  }

  public void updateClimb() {
    switch (climb.getIO().getCurrentState()) {
      case CLIMB:
        climbVis.setAngle(climb.getIO().getPositionInches());
        climbVis.setColor(new Color8Bit(Color.kGreen));
        System.out.println("ts pmo");
        break;
      case CLIMBREADY:
        climbVis.setAngle(Units.inchesToMeters(climb.getIO().getPositionInches()));
        climbVis.setColor(new Color8Bit(Color.kYellow));
        break;
      case STOW:
        climbVis.setAngle(Units.inchesToMeters(climb.getIO().getPositionInches()));
        climbVis.setColor(new Color8Bit(Color.kBlue));
        break;
      default:
        climbVis.setAngle(Units.inchesToMeters(climb.getIO().getPositionInches()));
        climbVis.setColor(new Color8Bit(Color.kBlue));
        break;
    }
  }

  public void updateEndefectorWrist() {
    switch (endefectorWrist.getIO().getCurrentState()) {
      case SCORING:
        endefectorWristVis.setAngle(endefectorWrist.getIO().getCurrentPositionDegrees());
        endefectorWristVis.setColor(new Color8Bit(Color.kRed));
        break;
      case GROUNDINTAKE:
        endefectorWristVis.setAngle(endefectorWrist.getIO().getCurrentPositionDegrees());
        endefectorWristVis.setColor(new Color8Bit(Color.kRed));
        break;
      case STATIONINTAKE:
        endefectorWristVis.setAngle(endefectorWrist.getIO().getCurrentPositionDegrees());
        endefectorWristVis.setColor(new Color8Bit(Color.kGreen));
        break;
      case CLIMB:
        endefectorWristVis.setAngle(endefectorWrist.getIO().getCurrentPositionDegrees());
        endefectorWristVis.setColor(new Color8Bit(Color.kRed));
        break;
      default: // STOW
        endefectorWristVis.setAngle(endefectorWrist.getIO().getCurrentPositionDegrees());
        endefectorWristVis.setColor(new Color8Bit(Color.kBlue));
        break;
    }
  }

  public void updateEndefectorRollers() {
    switch (endefectorRollers.getIO().getCurrentState()) {
      case STOP:
        endefectorRollersVis.setAngle(endefectorRollers.getIO().GetCurrentVolts());
        endefectorRollersVis.setColor(new Color8Bit(Color.kGreen));
        break;
      case SCORE:
        endefectorRollersVis.setAngle(endefectorRollers.getIO().GetCurrentVolts());
        endefectorRollersVis.setColor(new Color8Bit(Color.kRed));
        break;
      case INTAKE:
        endefectorRollersVis.setAngle((endefectorRollers.getIO().GetCurrentVolts()) * -1);
        endefectorRollersVis.setColor(new Color8Bit(Color.kPurple));
        break;
      default: // idle
        endefectorRollersVis.setAngle(endefectorRollers.getIO().GetCurrentVolts());
        endefectorRollersVis.setColor(new Color8Bit(Color.kYellow));
        break;
    }
  }
}
