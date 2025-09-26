package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static Mode getMode() {
    return currentMode;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM
  }

  public static final class kMotors {
    public static final class kKrakenX60Foc {
      public static final double FREE_SPEED = (608.0 / (2.0 * Math.PI)); // rad/s to rotations/s
      public static final double FREE_CURRENT = 2.0;
      public static final double STALL_TORQUE = 9.37;
      public static final double STALL_CURRENT = 483.0;

      public static final double kV = 12.0 / FREE_SPEED;
    }
  }
}
