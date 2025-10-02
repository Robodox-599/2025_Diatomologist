package frc.robot.subsystems.elevator;

public abstract class ElevatorIO {
  protected double tempCelsius = 0.0;
  protected double positionInches = 0.0;
  protected double velocityInchesPerSec = 0.0;
  protected double appliedVolts = 0.0;
  protected double currentAmps = 0.0;
  protected double targetPositionInches = 0.0;
  protected double acceleration = 0.0;
  protected boolean atSetpoint = false;

  protected double elevatorSoftLowerLimit = ElevatorConstants.elevatorHardLowerLimit;
  protected double elevatorSoftUpperLimit = ElevatorConstants.elevatorHardUpperLimit;

  /** Updates the set of loggable inputs */
  public void updateInputs() {}

  /** Sets the target height of the elevator */
  public void setHeight(ElevatorConstants.ElevatorStates state) {}

  /** Stops the elevator */
  public void stop() {}

  /** Sets brake mode */
  public void enableBrakeMode(boolean enable) {}

  public void zeroEncoder() {}

  public void setVoltage(double voltage) {}
}
