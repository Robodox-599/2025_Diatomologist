package frc.robot.subsystems.rollers;

public interface RollersIO {
    public default void runVelocity(double velocity){}
    public default void setVoltage(double voltage){}
    public default void stop(){ runVelocity(0);}
    public default void updateInputs(){}
}
