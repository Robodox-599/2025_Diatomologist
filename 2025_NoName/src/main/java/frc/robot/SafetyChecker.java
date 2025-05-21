package frc.robot;

import dev.doglog.DogLog;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants;

public class SafetyChecker {
  private double elevatorInches, wristDegrees;
  private final double maximumElevatorSwingThroughHeight =
      12.6; // maximum height that the endefector/elevator can be at so the endefector can swing
  // through the elevator
  private final double endefectorBehindElevatorDegrees =
      0.728; // the degrees threshold that the endefector is behind the elevator

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

  public boolean isSafeWrist(WristConstants.WristStates state) {
    switch (state) {
      case INTAKING_ALGAE_GROUND:
    }
    if (elevatorInches < maximumElevatorSwingThroughHeight) {
      return true;
    }
    return false;
  }

  public boolean isSafeElevator() {
    if (!isBehindElevator(wristDegrees)) { // if the wrist is NOT behind the elevator
      DogLog.log("SafetyChecker/isSafeElevator", true);
      return true;
    }
    DogLog.log("SafetyChecker/isSafeElevator", false);
    return false;
  }

  public boolean isBehindElevator(double wristSupplyDegrees) {
    return (endefectorBehindElevatorDegrees > wristSupplyDegrees);
  }
}
