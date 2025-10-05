package frc.robot.subsystems.endefector.endefectorrollers;

public abstract class RollersIO {
  protected double tempCelsius = 0.0;
  protected double statorCurrentAmps = 0.0;
  protected double supplyCurrentAmps = 0.0;

  protected double position = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double desiredVelocity = 0.0;
  protected boolean atSetpoint = false;

  protected boolean isCoralInRamp = false;
  protected boolean isCoralIntakedInEndefector = false;
  protected boolean isGroundAlgaeIntaked = false;
  protected boolean isReefAlgaeIntaked = false;
  protected boolean isCoralTroughScored = false;
  protected boolean isCoralBranchScored = false;
  protected boolean isAlgaeScored = false;
  protected double holdCoralPosition = 0.0;

  public void updateInputs() {}

  public void stop() {}

  public void setVelocity(RollersConstants.EndefectorRollerStates state) {}

  public void holdAlgae() {}

  public void holdCoral() {}

  public void setHoldCoralPosition() {}

  public double getVelocity() {
    return velocity;
  }

  public void resetRollersPosition() {}

  public void setCoralStateSim(boolean state) {}
}
