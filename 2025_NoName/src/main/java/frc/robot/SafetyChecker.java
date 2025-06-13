package frc.robot;

import dev.doglog.DogLog;

public class SafetyChecker {
  private double elevatorInches, wristDegrees;
  private final double maximumElevatorSwingThroughHeight =
      12.6; // maximum height that the elevator can be so the endefector can swing through the
  // elevator
  private final double minimumElevatorSwingAboveHeight =
      18.0; // minimum height that the elevator can be so the endefector can swing above the
  // elevator
  private final double endefectorBehindElevatorDegrees =
      0.728; // the degrees threshold that the endefector is behind the elevator
  private boolean isAtSetpointElevator = false;
  private boolean isAtSetpointWrist = false;

  public void setCurrentElevatorInches(
      double
          elevatorInches) { // set current elevator degrees, this should be done before every safe
    // check!
    this.elevatorInches = elevatorInches;
    DogLog.log("SafetyChecker/CurrentElevatorInches", this.elevatorInches);
  }

  public void setCurrentWristDegrees(
      double wristDegrees) { // set the current wrist degrees, this should be done before every safe
    // check!
    this.wristDegrees = wristDegrees;
    DogLog.log("SafetyChecker/CurrentWristDegrees", this.wristDegrees);
  }

  public void updateIsAtSetpointElevator(boolean value) {
    isAtSetpointElevator = value;
  }

  public void updateIsAtSetpointWrist(boolean value) {
    isAtSetpointWrist = value;
  }

  public boolean isSafeElevator() {
    if (!isBehindElevator(wristDegrees)
        || isUnderElevator(
            elevatorInches)) { // if the wrist is NOT behind the elevator or if the endefector is
      // BELOW
      // the elevator swing height, elevator is safe
      DogLog.log("SafetyChecker/isSafeElevator", true);
      return true;
    }
    DogLog.log("SafetyChecker/isSafeElevator", false);
    return false;
  }

  public boolean isSafeWrist() {
    if (isUnderElevator(elevatorInches)) {
      DogLog.log("SafetyChecker/isSafeWrist", true);
      return true;
    }
    DogLog.log("SafetyChecker/isSafeWrist", false);
    return false;
  }

  public boolean isBehindElevator(double wristSupplyDegrees) {
    return (endefectorBehindElevatorDegrees > wristSupplyDegrees);
  }

  public boolean isAboveElevator(double elevatorSupplyInches) {
    return (elevatorSupplyInches > minimumElevatorSwingAboveHeight);
  }

  public boolean isUnderElevator(double elevatorSupplyInches) {
    return (elevatorSupplyInches < maximumElevatorSwingThroughHeight);
  }

  public boolean isAtSetpoints() {
    DogLog.log("SafetyChecker/isAtSetpoints", isAtSetpointElevator && isAtSetpointWrist);
    return isAtSetpointElevator && isAtSetpointWrist;
  }
}
