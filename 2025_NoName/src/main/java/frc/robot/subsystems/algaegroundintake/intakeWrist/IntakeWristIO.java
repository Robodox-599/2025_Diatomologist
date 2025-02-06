package frc.robot.subsystems.algaegroundintake.intakewrist;

import frc.robot.util.AlgaeGroundIntakeUtil;

public abstract class IntakeWristIO {
  protected double tempCelsius = 0.0;
  protected double currentAmps = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double targetPosition = 0.0;
  protected double currentPosition = 0.0;
  protected double position = 0.0;
  protected boolean atSetpoint = false;
  protected IntakeWristConstants.AlgaeStates currentState = IntakeWristConstants.AlgaeStates.STOW;

  public void updateInputs() {}

  public void setVoltage(double voltage) {}

  public void goToPose(double position) {}

  public double getPose() {
    return 0.0;
  }

  public void stop() {}

  public void setBrake(boolean brake) {}

  public void setState(IntakeWristConstants.AlgaeStates state) {}

  public IntakeWristConstants.AlgaeStates getCurrentState() {
    return currentState;
  }

  public double getCurrentPosition() {
    return AlgaeGroundIntakeUtil.convertToTicks(currentPosition);
  }
}
