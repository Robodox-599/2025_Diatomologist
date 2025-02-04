package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;

public final class ElevatorConstants {
  /* CHANGE LATER */

  public static enum ElevatorStates {
    L1(1),
    L2(2),
    L3(3),
    L4(4),
    STOW(5),
    INTAKE(6),
    GROUNDINTAKE(7),
    ALGAE_L2(8),
    ALGAE_L3(9);
    private final int index;

    ElevatorStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  // Setpoint positions in encoder ticks or inches
  public static final double[] heights = {
    0.0, // L1
    20.0, // L2
    40.0, // L3
    60.0, // L4
    0.3 // STOW
  };

  public static final int leaderMotorID = 20;
  public static final String leaderMotorCANbus = "rio";
  public static final int followerMotorID = 17;
  public static final String followerMotorCANbus = "rio";
  public static final boolean followerInverted = true;

  public static final int limitSwitchDioPort = 0;
  public static final double gearRatio = 10.0;
  public static final double drumCircumferenceInches = 1.8;
  public static final double inchesPerRev = 10; // idk prolly wrong
  public static final double drumRadiusMeters = Units.Inches.of(2).magnitude();

  public static final double maxVelocityInchesPerSec = 60.0;
  public static final double maxAccelerationInchesPerSecSQ = 120.0;

  /* Position Setpoints (in inches) */
  public static final double elevatorLowerLimit = 0.0;
  public static final double elevatorUpperLimit = 61.0;
  public static final double homePositionOffset = 1.0;

  public static final double levelOneHeight = 0.0;
  public static final double levelTwoHeight = 12.0;
  public static final double levelThreeHeight = 24.0;
  public static final double levelFourHeight = 72.0;
  public static final double stowHeight = 0;
  public static final double endEffectorIntakeHeight = 0;
  public static final double groundIntakeHeight = 0;

  public static final double kP = 1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.0;
  public static final double kS = 0.0;

  // Add these for better PID tuning
  public static final double simkP = 0.8;
  public static final double simkI = 0.001;
  public static final double simkD = 0.05;
  public static final double simkF = 0.0;

  public static final double supplyCurrentLimitAmps = 0.1;
  public static final double statorCurrentLimitAmps = 0.1;

  public static final double PositionToleranceInches = 0.5;
  public static final double velocityToleranceInchesPerSec = 0.5;

  public static final double elevatorMOI = 0.15;
  public static final double nominal_voltage = 12.0;

  public static final int movingUpSlot = 1;
  public static final int movingDownSlot = 0;
}
