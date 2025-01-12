package frc.robot.Subsystems.Elevator;

public interface ElevatorIO {
    public static class ElevatorInputs {
        public double positionInches = 0.0;
        public double velocityInchesPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double targetPositionInches = 0.0;
        public boolean hallEffectTriggered = false;
        public boolean atSetpoint = false;
        public ElevatorState state = ElevatorState.DISABLED;
        public double tempCelsius = 0.0;
    }

    public enum ElevatorState {
        HOMING, MOVING, HOLDING, DISABLED, ERROR
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(ElevatorInputs inputs) {}

    /** Sets the target height of the elevator */
    public default void setHeight(double heightInches) {}

    /** Runs motion magic at voltage */
    public default void MotionMagicVoltage(double volts) {}

    /** Sets encoder position */
    public default void setEncoder(double positionInches) {}

    /** Stops the elevator */
    public default void stop() {}

    /** Sets brake mode */
    public default void enableBrakeMode(boolean enable) {}

    /** Zeros encoder */
    public default void zeroEncoder() {}

    /** Configure PID */
    public default void configurePID(double kP, double kI, double kD) {}
}