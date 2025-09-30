package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants;

public class SubsystemChecker extends SubsystemBase {
  private Elevator elevator;
  private Wrist wrist;

  public void setElevator(Elevator elevator) {
    this.elevator = elevator;
  }

  public void setWrist(Wrist wrist) {
    this.wrist = wrist;
  }

  private final double maximumElevatorSwingThroughHeight =
      12.4; // max height of elevator where endefector can safely swing through
  private final double minimumElevatorSwingAboveHeight =
      28.0; // min height of elevator where endefector can safely swing behind
  private final double minimumElevatorSwingBelowHeight =
      7.0; // min height of elevator where endefector can safely swing below
  private final double endefectorBehindElevatorPosition =
      SubsystemUtil.wristStateToSetpoint(WristConstants.WristStates.POSITION_PREPARED)
          - WristConstants
              .wristPositionTolerance; // any wrist position less than this is behind the elevator
  private final double endefectorBeyondHorizontalPosition = 0.05;

  public double calculateElevatorSoftLowerLimit() {
    // if (wrist.getPosition() > endefectorBeyondHorizontalPosition
    //     && elevator.getHeightInches() > minimumElevatorSwingBelowHeight) {
    //   return minimumElevatorSwingBelowHeight;
    // return minimumElevatorSwingBelowHeight * (Math.sin(-2 * wrist.getPosition())); <- this
    // would make the limit dynamic based on wrist position
    // } else
    if (wrist.getPosition() < endefectorBehindElevatorPosition
        && elevator.getHeightInches() > minimumElevatorSwingAboveHeight) {
      return minimumElevatorSwingAboveHeight;
    } else {
      return ElevatorConstants.elevatorHardLowerLimit;
    }
  }

  public double calculateElevatorSoftUpperLimit() {
    if (wrist.getPosition() < endefectorBehindElevatorPosition
        && elevator.getHeightInches() < maximumElevatorSwingThroughHeight) {
      return maximumElevatorSwingThroughHeight;
    } else {
      return ElevatorConstants.elevatorHardUpperLimit;
    }
  }

  public boolean isEndefectorUnderElevator() {
    if ((elevator.getHeightInches() < maximumElevatorSwingThroughHeight)) {
      DogLog.log("SafetyChecker/isEndefectorUnderElevator", true);
      return true;
    }
    DogLog.log("SafetyChecker/isEndefectorUnderElevator", false);
    return false;
  }

  public boolean isAtHeightElevator(ElevatorConstants.ElevatorStates state) {
    return elevator.isAtSetpoint(state);
  }

  public boolean isAtPositionWrist(WristConstants.WristStates state) {
    return wrist.isAtSetpoint(state);
  }

  // public boolean isSafeElevator() {
  //   if (!isBehindElevator(wrist.getPosition())
  //       || isUnderElevator(
  //           elevator.getHeightInches())) { // if the wrist is NOT behind the elevator or if the
  //     // endefector is
  //     // BELOW
  //     // the elevator swing height, elevator is safe
  //     DogLog.log("SafetyChecker/isSafeElevator", true);
  //     return true;
  //   }
  //   DogLog.log("SafetyChecker/isSafeElevator", false);
  //   return false;
  // }

  // public boolean isEndefectorBehindElevator() {
  //   DogLog.log("SafetyChecker/isEndefectorBehindElevator", isBehindElevator(wristDegrees));
  //   return isBehindElevator(wristDegrees);
  // }

  // public boolean isBehindElevator(double wristSupplyDegrees) {
  //   return (endefectorBehindElevatorDegrees > wristSupplyDegrees);
  // }

  // public boolean isAboveElevator(double elevatorSupplyInches) {
  //   return (elevatorSupplyInches > minimumElevatorSwingAboveHeight);
  // }

  // public boolean isReadyToScore() {
  //   DogLog.log("SafetyChecker/readyToScore", isAtSetpointElevator && isAtSetpointWrist);
  //   return isAtSetpointElevator && isAtSetpointWrist;
  // }

  // public boolean isAtSetpointElevator() {
  //   return isAtSetpointElevator;
  // }

  // public boolean isWristAtPrepared() {
  //   return Math.abs(wristDegrees - 0.79) < WristConstants.wristPositionTolerance;
  // }
}
