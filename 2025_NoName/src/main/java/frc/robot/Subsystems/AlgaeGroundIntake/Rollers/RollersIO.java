package frc.robot.subsystems.algaegroundintake.rollers;

public abstract class RollersIO {
    protected double velocityRadsPerSec = 0.0;
    protected double appliedVoltage = 0.0;
    protected double velocitySetpoint = 0.0;
    protected double currentAmps = 0.0;
    protected double tempCelcius = 0.0;

    public void updateInputs() {}

    public void setVoltage(double voltage) {}

    public void setVelocity(double velocity) {}

    public void setBrake(boolean brake) {}

    public void stop() {
        setVoltage(0.0);
    }
}
