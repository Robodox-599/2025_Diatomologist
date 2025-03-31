package frc.robot.subsystems.endefector.endefectorrollers;

public abstract class RollersIO {
  protected double tempCelsius = 0.0;
  protected double currentAmps = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double desiredVelocity = 0.0;
  protected double canrangeDistance = 0.0;
  protected boolean atSetpoint = false;
  protected boolean isAlgaeDetected = false;
  protected RollersConstants.EndefectorRollerStates currentState =
      RollersConstants.EndefectorRollerStates.STOP;

  public void updateInputs() {}

  public void stop() {}

  public void setVelocity(double velocity) {}

  public void setState(RollersConstants.EndefectorRollerStates state) {}

  public void holdCoral() {}

  public void holdCoralAfterIntake() {}

  public void holdAlgae() {}

  public double getCurrentVolts() {
    return appliedVolts;
  }

  public boolean isCoralDetected() {
    return false;
  }

  public boolean isAlgaeDetected() {
    return false;
  }

  public boolean isRollersStalling() {
    return false;
  }

  public RollersConstants.EndefectorRollerStates getState() {
    return currentState;
  }

  // public double getCoralDistance() {
  //   return 0.0;
  // }

  public double getVelocity() {
    return velocity;
  }

  // public void setBrake(boolean brake) {}

  // public void setSpeed(double speed) {}

  // public void setVoltage(double voltage) {}
}
