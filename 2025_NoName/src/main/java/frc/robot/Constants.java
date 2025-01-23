package frc.robot;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static int driverControllerPort = 0;

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static Mode getMode() {
        return currentMode;
      }
    
      public static enum Mode {
        REAL,
        SIM
      }
}