package frc.robot.subsystems.endefector.endefectorrollers;

public abstract class RollersIO {
  protected double tempCelsius = 0.0;
  protected double statorCurrentAmps = 0.0;
  protected double supplyCurrentAmps = 0.0;

  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double desiredVelocity = 0.0;
  protected boolean atSetpoint = false;
  protected boolean isAlgaeDetected = false;
  protected boolean isCoralDetected = false;

  protected RollersConstants.EndefectorRollerStates currentState =
      RollersConstants.EndefectorRollerStates.STOP;

  public void updateInputs() {}

  public void stop() {}

  public void setVelocity(double velocity) {}

  public void setState(RollersConstants.EndefectorRollerStates state) {}

  public void adjustCoralAfterStationIntake() {}

  public void holdAlgae() {}

  public RollersConstants.EndefectorRollerStates getState() {
    return currentState;
  }

  public double getVelocity() {
    return velocity;
  }
}
