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
      public static final double FREE_SPEED = (608.0 / (2.0 * Math.PI));
      public static final double FREE_CURRENT = 2.0;
      public static final double STALL_TORQUE = 9.37;
      public static final double STALL_CURRENT = 483.0;

      public static final double kV = 12.0 / FREE_SPEED;
    }
  }

  // real pid

  // .5 to mvoe up
  // .3 to move down
  // subtract the two (0.5-0.3 = 0.2) and divide the difference by 2, (0.2/2 = 0.1)
  // kS = 0.3 + (0.2/2)
  // kG = 0.2/2
  // goin up = kS+kG
  // goin down = kG-kS

  // max velocity on elevator is your kV times twelve
  // max acceleration on elevator is the time it takes to accelerate, and i take my velocity divided
  // by that time. boom max accel
  // my max velocity is my freespeed minus my speed loss, speed loss calculation =
  // (12+kS+kG)/freeSpeed
  //

}
