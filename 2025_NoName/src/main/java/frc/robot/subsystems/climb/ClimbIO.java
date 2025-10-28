package frc.robot.subsystems.climb;

public abstract class ClimbIO {
  protected double rollersTempCelsius = 0.0;
  protected double rollersVelocity = 0.0;
  protected double rollersAppliedVolts = 0.0;
  protected double rollersCurrentAmps = 0.0;
  protected double rollersStatorCurrent = 0.0;

  protected double climbTempCelsius = 0.0;
  protected double climbPosition = 0.0;
  protected double climbVelocity = 0.0;
  protected double climbAppliedVolts = 0.0;
  protected double climbCurrentAmps = 0.0;
  protected double targetPositionDegrees = 0.0;

  protected boolean isRampReleased = false;
  protected boolean isCageDetected = false;

  protected boolean atSetpoint = false;

  public void updateInputs() {}

  public void setRollersVelocity(double velocity) {}

  public void stallRollers() {}

  public void releaseRamp() {}

  public void stop() {}

  public void enableBrakeMode(boolean enable) {}

  public void setClimbVoltage(double voltage) {}

  public void zeroEncoder() {}

  public double GetClimbCurrentVolts() {
    return climbAppliedVolts;
  }

  public double GetRollersCurrentVolts() {
    return rollersAppliedVolts;
  }
}
