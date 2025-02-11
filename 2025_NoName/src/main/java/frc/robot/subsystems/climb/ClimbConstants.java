package frc.robot.subsystems.climb;

import edu.wpi.first.units.Units;

public class ClimbConstants {
  /* CHANGE LATER */

  public static enum ClimbStates {
    CLIMBREADY(1),
    CLIMB(2),
    STOW(3);

    private final int index;

    ClimbStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  // Setpoint positions in encoder ticks or inches
  public static final double[] setpoint = {
    3.5, // stow
    10.5, // climb
    30.0 // ready to climb
  };

  public static final int leaderMotorID = 20;
  public static final String leaderMotorCANbus = "rio";
  public static final int followerMotorID = 17;
  public static final String followerMotorCANbus = "rio";
  public static final boolean followerInverted = true;

  public static final int limitSwitchDioPort = 0;
  public static final double gearRatio = 6;
  public static final double drumCircumferenceInches = 1.8;
  public static final double inchesPerRev = 0.39; // idk prolly wrong
  public static final double drumRadiusMeters = Units.Inches.of(2).magnitude();

  public static final double maxVelocityInchesPerSec = 60.0;
  public static final double maxAccelerationInchesPerSecSQ = 120.0;

  /* Position Setpoints (in inches) */
  public static final double climbLowerLimit = 0.0;
  public static final double climbUpperLimit = 48.0;
  public static final double homePositionOffset = 1.0;

  public static final double climbHeight = 30.0;
  public static final double stowHeight = 0;
  public static final double endEffectorIntakeHeight = 0;
  public static final double groundIntakeHeight = 0;

  public static final double kP = 1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.0;
  public static final double kS = 0.0;

  public static final double simkP = 8;
  public static final double simkI = 0.005;
  public static final double simkD = 0.0;
  public static final double simkF = 0.0;

  public static final double supplyCurrentLimitAmps = 0.1;
  public static final double statorCurrentLimitAmps = 0.1;

  public static final double PositionToleranceInches = 0.5;
  public static final double velocityToleranceInchesPerSec = 0.5;

  public static final double climbMOI = 0.15;
  public static final double nominal_voltage = 12.0;

  public static final int movingUpSlot = 1;
  public static final int movingDownSlot = 0;
}
