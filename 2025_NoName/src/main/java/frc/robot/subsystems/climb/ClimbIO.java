package frc.robot.subsystems.climb;

import frc.robot.subsystems.climb.ClimbConstants.ClimbStates;

public abstract class ClimbIO {
  protected double rollersTempCelsius = 0.0;
  protected double rollersVelocity = 0.0;
  protected double rollersAppliedVolts = 0.0;
  protected double rollersCurrentAmps = 0.0;
  protected double rollersStatorCurrent = 0.0;

  protected double climbTempCelsius = 0.0;
  protected double climbPositionDegrees = 0.0;
  protected double climbVelocity = 0.0;
  protected double climbAppliedVolts = 0.0;
  protected double climbCurrentAmps = 0.0;
  protected double targetPositionDegrees = 0.0;

  protected boolean isCageDetected = false;

  protected boolean atSetpoint = false;

  public void updateInputs() {}

  public void setClimbPosition(ClimbStates state) {}

  public void setRollersVelocity(double velocity) {}

  public void stallRollers() {}

  public void holdRampServo() {}

  public void releaseRampServo() {}

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
