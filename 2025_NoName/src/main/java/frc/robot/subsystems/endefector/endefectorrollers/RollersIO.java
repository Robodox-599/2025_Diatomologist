package frc.robot.subsystems.endefector.endefectorrollers;

public abstract class RollersIO {
  protected double endefectorRollersPosition = 0.0;
  protected double endefectorRollersAppliedVolts = 0.0;
  protected double endefectorRollersVelocity = 0.0;
  protected double endefectorRollersTempCelsius = 0.0;
  protected double endefectorRollersStatorCurrent = 0.0;
  protected double endefectorRollersSupplyCurrent = 0.0;

  protected double rampRollersPosition = 0.0;
  protected double rampRollersAppliedVolts = 0.0;
  protected double rampRollersVelocity = 0.0;
  protected double rampRollersTempCelsius = 0.0;
  protected double rampRollersStatorCurrent = 0.0;
  protected double rampRollersSupplyCurrent = 0.0;

  protected boolean isCoralInRamp = false;
  protected boolean isCoralIntakedInEndefector = false;
  protected boolean isCoralInTransition = false;

  protected boolean isGroundAlgaeIntaked = false;
  protected boolean isReefAlgaeIntaked = false;
  protected boolean isCoralTroughScored = false;
  protected boolean isCoralBranchScored = false;
  protected boolean isAlgaeScored = false;

  protected double endefectorHoldCoralPosition = 0.0;
  protected double rampHoldCoralPosition = 0.0;

  public void updateInputs() {}

  public void stop() {}

  public void setEndefectorVelocity(RollersConstants.EndefectorRollerStates state) {}

  public void setRampVelocity(double speed) {}

  public void holdAlgae() {}

  public void rampHoldCoral() {}

  public void endefectorHoldCoral() {}

  public void setRampHoldCoralPosition() {}

  public void setEndefectorHoldCoralPosition() {}

  public void resetRollersPosition() {}

  public void setCoralStateSim(boolean state) {}
}
