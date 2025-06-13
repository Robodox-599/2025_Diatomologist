package frc.robot.subsystems.climb;

public class ClimbConstants {
  //motor information
  public static final int climbMotorID = 20;
  public static final String climbMotorCANbus = "rio";
  public static final int rollersMotorID = 0;
  public static final String rollersMotorCANbus = "rio";
  public static final double climbMOI = 0.15;
  public static final double gearRatio = 5.4;

  //current limits
  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  //pid values
  public static final double kP = 1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.0;
  public static final double kS = 0.0;
  //sim values
  public static final double simkP = 8.5;
  public static final double simkI = 0.005;
  public static final double simkD = 0.0;
  public static final double simkF = 0.0;

  //setpoints
  public static final double climbLowerLimit = 0.0;
  public static final double climbUpperLimit = 48.0;
  public static final double homePositionOffset = 1.0;

  //climb states
  public static enum ClimbStates {
    CLIMB_READY(0),
    CLIMB(1),
    STOPPED(2);

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

  public static final double[] setpoint = {
    3.5, // ready to climb
    10.5, // climb
    30.0 // stopped
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
