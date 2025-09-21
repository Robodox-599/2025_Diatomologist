package frc.robot.subsystems.endefector.endefectorrollers;

public abstract class RollersIO {
  protected double tempCelsius = 0.0;
  protected double statorCurrentAmps = 0.0;
  protected double supplyCurrentAmps = 0.0;

  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double desiredVelocity = 0.0;
  protected boolean atSetpoint = false;

  protected boolean isCoralInRamp = false;
  protected boolean isCoralIntakedInEndefector = false;
  protected boolean isAlgaeIntaked = false;
  protected boolean isCoralTroughScored = false;
  protected boolean isCoralBranchScored = false;
  protected boolean isAlgaeScored = false;

  public void updateInputs() {}

  public void stop() {}

  public void setVelocity(RollersConstants.EndefectorRollerStates state) {}

  public void grabOrHoldAlgae() {}

  public double getVelocity() {
    return velocity;
  }

  public void setCoralStateSim(boolean state) {}
}
