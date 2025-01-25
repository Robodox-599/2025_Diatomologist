package frc.robot.subsystems.algaegroundintake.rollers;

public abstract class RollersIO {

    public void updateInputs() {}

    public void setVoltage(double voltage) {}

    public void setVelocity(double velocity) {}

    public void setBrake(boolean brake) {}

    public void stop() {
        setVoltage(0.0);
    }
}
