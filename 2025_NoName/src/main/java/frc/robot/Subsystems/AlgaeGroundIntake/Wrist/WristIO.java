package frc.robot.subsystems.algaegroundintake.wrist;

public interface WristIO {
    public abstract class WristIOInputs {
        // protected double angularRads = 0.0;
        // protected double getAngularVelocityRadPerSec = 0.0;
        // protected double appliedVoltage = 0.0;
        // protected double setpointAngleRads = 0.0;
        // protected double currentAmps = 0.0;
        // protected double tempCelcius = 0.0;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void goToPose(double position) {}

    public default double getPose() {
        return 0.0;
    }
    public default void setSpeed(double speed) {}

    public default void setBrake(double brake) {}

    public default double getAngle() {
        return 0.0;
    }

    public default void stop() {}

    public default void goToSetpoint(double setpoint) {}

    public default void holdToSetpoint(double setpoint) {}

    public default void setBrake(boolean brake) {}

    public default boolean atSetpoint() {
        return false;
    }

    public default double getP() {
        return 0.0;
    }

    public default double getI() {
        return 0.0;
    }

    public default double getD() {
        return 0.0;
    }

    public default double getFF() {
        return 0.0;
    }

    public default double getkS() {
        return 0.0;
    }

    public default double getkG() {
        return 0.0;
    }

    public default double getkV() {
        return 0.0;
    }

    public default double getkA() {
        return 0.0;
    }

    public default void setI(double i) {}

    public default void setD(double d) {}

    public default void setFF(double ff) {}

    public default void setkS(double kS) {}

    public default void setkV(double kV) {}

    public default void setkG(double kG) {}

    public default void setkA(double kA) {}

    public default void setP(double p) {}
}
