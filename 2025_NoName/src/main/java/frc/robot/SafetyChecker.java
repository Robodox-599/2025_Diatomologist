package frc.robot;

public class SafetyChecker {
  private double elevatorMeters, wristDegrees;
  private final double maximumElevatorSwingThroughHeight =
      0.0; // maximum hieght that the endefector/ elevator can be at so the endefector can swing
  // through the elevator
  private final double endefectorBehindElevatorDegrees =
      76.0; // the degrees threshold that the endefector is behind the elevator, found in cad

  public void setCurrentElevatorMeters(double elevatorMeters) {
    this.elevatorMeters = elevatorMeters;
  }

  public void setCurrentWristDegrees(double wristDegrees) {
    this.wristDegrees = wristDegrees;
  }

  public boolean isSafeWrist(double wristTargetDegrees) {
    boolean targetBehindElevator = isBehindElevator(wristTargetDegrees);
    boolean currentBehindElevator = isBehindElevator(wristDegrees);
    if (targetBehindElevator != currentBehindElevator) {
      // if the intake is swinging through elevator
      return elevatorMeters < maximumElevatorSwingThroughHeight;
    }
    return true;
  }

  public boolean isSafeElevator(double elevatorTargetMeters) {
    if (isBehindElevator(wristDegrees)
        && elevatorTargetMeters < maximumElevatorSwingThroughHeight) {
      // if the intake is swinging through elevator
      return true;
    } else if (!isBehindElevator(wristDegrees)) {
      // if the intake is not in front of elevator
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
