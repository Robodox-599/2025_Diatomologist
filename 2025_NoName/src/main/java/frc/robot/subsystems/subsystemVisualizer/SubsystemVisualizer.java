package frc.robot.subsystems.subsystemVisualizer;

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

  Mechanism2d mech = new Mechanism2d(60, 60);
  MechanismRoot2d root = mech.getRoot("root", 30, 0);

  MechanismLigament2d elevatorVis =
      root.append(new MechanismLigament2d("elevator", 10, 90, 8, new Color8Bit(Color.kRed)));

  MechanismLigament2d endefectorWristVis =
      elevatorVis.append(
          new MechanismLigament2d("endefectorWristVis", 6, -45, 4, new Color8Bit(Color.kPurple)));

  MechanismLigament2d endefectorRollersVis =
      endefectorWristVis.append(
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
    updateWrist();
    updateRollers();

    endefectorRollersVis.setAngle(45);

    SmartDashboard.putData("DongleMechanism2D", mech);
  }

  public void updateElevator() {
    elevatorVis.setLength(Units.inchesToMeters(elevator.getPositionInches()) * 25);
  }

  public void updateWrist() {
    endefectorWristVis.setAngle(
        Units.inchesToMeters(endefectorWrist.getAngle()) * 25);
  }

  public void updateRollers() {
    endefectorRollersVis.setAngle(
        Units.inchesToMeters(endefectorRollers.getVelocity()) * 300);
  }
}
