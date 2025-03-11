package frc.robot;

import dev.doglog.DogLog;

public class SafetyChecker {
  private double elevatorInches, wristDegrees;
  private final double maximumElevatorSwingThroughHeight =
      12.6; // maximum height that the endefector/elevator can be at so the endefector can swing
  // through the elevator, check cad for this
  private final double endefectorBehindElevatorDegrees =
      0.728; // the degrees threshold that the endefector is behind the elevator, found in cad

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

  public boolean isSafeWrist(double wristTargetDegrees) {

    boolean targetBehindElevator =
        isBehindElevator(
            wristTargetDegrees); // is the target behind the elevator allocated degrees?
    boolean currentBehindElevator =
        isBehindElevator(
            wristDegrees); // is the current angle behind the elevator allocated degrees?
    DogLog.log("SafetyChecker/isSafeWrist/CurrentBehindElevator", currentBehindElevator);
    DogLog.log("SafetyChecker/isSafeWrist/TargetBehindElevator", targetBehindElevator);
    if (targetBehindElevator != currentBehindElevator) {
      // if one is not equal to the other, then check the make sure the height of the elevator is
      // good enough to allow swing through the elevator.
      DogLog.log(
          "SafetyChecker/isSafeWrist", this.elevatorInches < maximumElevatorSwingThroughHeight);
      return this.elevatorInches < maximumElevatorSwingThroughHeight;
    }
    DogLog.log("SafetyChecker/isSafeWrist", true);
    return true; // if both are equal, then just check the height of the elevator
  }

  // public boolean isInsideElevator(double wristTargetDegrees) {
  //   boolean targetBehindElevator =
  //       isBehindElevator(
  //           wristTargetDegrees); // is the target behind the elevator allocated degrees?
  //   boolean currentBehindElevator =
  //       isBehindElevator(
  //           wristDegrees); // is the current angle behind the elevator allocated degrees?
  //   DogLog.log("SafetyChecker/isInsideElevator/CurrentBehindElevator", currentBehindElevator);
  //   DogLog.log("SafetyChecker/isInsideElevator/TargetBehindElevator", targetBehindElevator);
  //   if (targetBehindElevator == currentBehindElevator
  //       && maximumElevatorSwingThroughHeight < elevatorInches) {
  //     // if the target is behind the elevator and the current is behind the elevator and the
  //     // elevator is under the max swing through height, then return true
  //     DogLog.log("SafetyChecker/isInsideElevator", true);
  //     return true;
  //   }
  //   DogLog.log("SafetyChecker/isInsideElevator", false);
  //   return false;
  // }

  public boolean isSafeElevator(double elevatorTargetInches) {
    if (isBehindElevator(wristDegrees) && elevatorTargetInches < maximumElevatorSwingThroughHeight
        || (!isBehindElevator(wristDegrees))) {
      // if the endefector is swinging through elevator but the setpoint is under the max elevator
      // swing through height
      // or if the endefector is not behind the elevator, then return true
      DogLog.log("SafetyChecker/isSafeElevator", true);
      return true;
    }
    DogLog.log("SafetyChecker/isSafeElevator", false);
    return false;
  }

  public boolean isBehindElevator(double wristSupplyDegrees) {
    return (endefectorBehindElevatorDegrees
        > wristSupplyDegrees); // should just be a greater than check
  }
}

// pass saftey checker into the elevator and wrist,
