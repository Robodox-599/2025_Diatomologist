package frc.robot;

public class SafetyChecker {
  private double elevatorInches, wristDegrees;
  private final double maximumElevatorSwingThroughHeight =
      0.0; // maximum height that the endefector/elevator can be at so the endefector can swing
  // through the elevator, check cad for this
  private final double maxHitCrossbarHeight =
      0.0; // maximum height that the endefector/elevator can be at so the endefector can swing
  // through the elevator, check cad for this
  private final double endefectorBehindElevatorDegrees =
      76.0; // the degrees threshold that the endefector is behind the elevator, found in cad

  public void setCurrentElevatorInches(
      double
          elevatorInches) { // set current elevator degrees, this should be done before every safe
    // check!
    this.elevatorInches = elevatorInches;
  }

  public void setCurrentWristDegrees(
      double wristDegrees) { // set the current wrist degrees, this should be done before every safe
    // check!
    this.wristDegrees = wristDegrees;
  }

  public boolean isSafeWrist(double wristTargetDegrees) {
    boolean targetBehindElevator =
        isBehindElevator(
            wristTargetDegrees); // is the target behind the elevator allocated degrees?
    boolean currentBehindElevator =
        isBehindElevator(
            wristDegrees); // is the current angle behind the elevator allocated degrees?
    if (targetBehindElevator != currentBehindElevator) {
      // if one is not equal to the other, then check the make sure the height of the elevator is
      // good enough to allow swing through the elevator.
      return elevatorInches < maximumElevatorSwingThroughHeight;
    } else if (isInsideElevator(wristTargetDegrees)) {
      return false;
    }
    return true; // if both are equal, then just check the height of the elevator
  }

  public boolean isInsideElevator(double wristTargetDegrees) {
    boolean targetBehindElevator =
        isBehindElevator(
            wristTargetDegrees); // is the target behind the elevator allocated degrees?
    boolean currentBehindElevator =
        isBehindElevator(
            wristDegrees); // is the current angle behind the elevator allocated degrees?
    if (targetBehindElevator == currentBehindElevator && maxHitCrossbarHeight < elevatorInches) {
      // if the target is behind the elevator and the current is behind the elevator and the
      // elevator is under the max swing through height, then return true
      return true;
    }
    return false;
  }

  public boolean isSafeElevator(double elevatorTargetInches) {
    if (isBehindElevator(wristDegrees) && elevatorTargetInches < maximumElevatorSwingThroughHeight
        || (!isBehindElevator(wristDegrees))) {
      // if the endefector is swinging through elevator but the setpoint is under the max elevator
      // swing through height
      // or if the endefector is not behind the elevator, then return true
      return true;
    }
    return false;
  }

  public boolean isBehindElevator(double wristSupplyDegrees) {
    return (endefectorBehindElevatorDegrees
        > wristSupplyDegrees); // should just be a greater than check
  }
}

// pass saftey checker into the elevator and wrist,
