// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

public abstract class ClimbIO {
    protected double tempCelsius = 0.0;
    protected double positionInches = 0.0;
    protected double velocityInchesPerSec = 0.0;
    protected double appliedVolts = 0.0;
    protected double currentAmps = 0.0;
    protected double targetPositionInches = 0.0;
    protected boolean limitSwitchValue = false;
    protected boolean atSetpoint = false;
    protected ClimbConstants.ClimbStates state = ClimbConstants.ClimbStates.STOW;

    /** Updates the set of loggable inputs */
    public void updateInputs() {}

    /** Sets the target height of the elevator */
    public void setState(ClimbConstants.ClimbStates state) {}

    /** Stops the elevator */
    public void stop() {}

    /** Sets brake mode */
    public void enableBrakeMode(boolean enable) {}

    public void zeroEncoder(){}

    public void setVoltage(double voltage){}

    public double getPosition(){return 0.0;}
}
