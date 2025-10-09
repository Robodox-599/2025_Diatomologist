package frc.robot.subsystems.climb;

public class ClimbConstants {
  // motor information
  public static final int climbMotorID = 20;
  public static final String climbMotorCANbus = "rio";
  public static final int rollersMotorID = 0;
  public static final String rollersMotorCANbus = "rio";
  public static final double climbMOI = 0.15;
  public static final double rollersMOI = 0.04;
  public static final double gearRatio = 5.4;

  // servos and limit switches
  // public static final int rollersLimitSwitchDioPort = 0;
  public static final int deployLimitSwitchDioPort = 1;
  // public static final int climbLimitSwitchDioPort = 2;
  public static final int flapServoPWMPort1 = 0;
  public static final int flapServoPWMPort2 = 1;
  public static final int rampServoPWMPort1 = 2;
  public static final int rampServoPWMPort2 = 3;

  // current limits
  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  // pid values
  public static final double kP = 1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.0;
  public static final double kS = 0.0;
  // sim values
  public static final double simkP = 8.5;
  public static final double simkI = 0.005;
  public static final double simkD = 0.0;
  public static final double simkF = 0.0;

  // setpoints
  public static final double climbLowerLimit = 0.0;
  public static final double climbUpperLimit = 48.0;
  public static final double homePositionOffset = 1.0;
  public static final double stallRollersVoltage = 0.1;

  // climb states
  public static enum ClimbStates {
    DEPLOYING_CLIMB(0),
    CLIMBING_UP(1),
    CLIMBING_DOWN(2),
    STOPPED(3);

    private final int index;

    ClimbStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public static final int movingUpSlot = 1;
  public static final int movingDownSlot = 0;

  public static final double[] voltage = {
    6.0, // deploying climb
    11.0, // climbing up
    -3.0, // climbing down
    0.0 // stowed
  };

  // public static final double maxVelocityInchesPerSec = 60.0;
  // public static final double maxAccelerationInchesPerSecSQ = 120.0;
  // public static final double velocityToleranceInchesPerSec = 2.0;
  // public static final double positionToleranceInches = 0.5;

  // public static final int limitSwitchDioPort = 0;
  // public static final double drumCircumferenceInches = Math.PI * 2.0;
  // public static final double inchesPerRev =
  //     drumCircumferenceInches / gearRatio; // reduction so dividing by gear ratio
  // public static final double drumRadiusMeters = Units.Inches.of(2).magnitude();

  // public static final double climbHeight = 30.0;
  // public static final double climbReadyHeight = 15.0;
  // public static final double stowHeight = 6.0;
  // public static final double endEffectorIntakeHeight = 0;
  // public static final double groundIntakeHeight = 0;
}
