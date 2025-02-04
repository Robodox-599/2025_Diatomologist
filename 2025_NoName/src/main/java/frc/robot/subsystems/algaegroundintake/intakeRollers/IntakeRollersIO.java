package frc.robot.subsystems.algaegroundintake.intakeRollers;

public abstract class IntakeRollersIO {
  protected double tempCelsius = 0.0;
  protected double currentAmps = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double desiredVelocity = 0.0;
  protected IntakeRollersConstants.AlageRollerStates currentState =
      IntakeRollersConstants.AlageRollerStates.STOP;

  public void updateInputs() {}

  public void setVoltage(double voltage) {}

  public void setVelocity(double velocity) {}

  public void setBrake(boolean brake) {}

  public void stop() {
    setVoltage(0.0);
  }

  public IntakeRollersConstants.AlageRollerStates getCurrentState() {
    return currentState;
  }

  public void setState(IntakeRollersConstants.AlageRollerStates state) {
    currentState = state;
  }

  public double GetCurrentVolts() {
    return appliedVolts;
  }
}
