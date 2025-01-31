package frc.robot.subsystems.algaegroundintake.intakeWrist;

public abstract class IntakeWristIO {
    protected double tempCelsius = 0.0;
  protected double currentAmps = 0.0;
  protected double appliedVolts = 0.0;
  protected double velocity = 0.0;
  protected double targetPosition = 0.0;
  protected double currentPosition = 0.0;
  protected double position = 0.0;

    public   void updateInputs() {}

    public   void setVoltage(double volts) {}

    public   void goToPose(double position) {}

    public   double getPose() {
        return 0.0;
    }
    public   void setSpeed(double speed) {}

    public   void setBrake(double brake) {}

    public   double getAngle() {
        return 0.0;
    }

    public   void stop() {}

    public   void goToSetpoint(double setpoint) {}

    public   void holdToSetpoint(double setpoint) {}

    public   void setBrake(boolean brake) {}

    public   boolean atSetpoint() {
        return false;
    }

    public   double getP() {
        return 0.0;
    }

    public   double getI() {
        return 0.0;
    }

    public   double getD() {
        return 0.0;
    }

    public   double getFF() {
        return 0.0;
    }

    public   double getkS() {
        return 0.0;
    }

    public   double getkG() {
        return 0.0;
    }

    public   double getkV() {
        return 0.0;
    }

    public   double getkA() {
        return 0.0;
    }

    public   void setI(double i) {}

    public   void setD(double d) {}

    public   void setFF(double ff) {}

    public   void setkS(double kS) {}

    public   void setkV(double kV) {}

    public   void setkG(double kG) {}

    public   void setkA(double kA) {}

    public   void setP(double p) {}
}
