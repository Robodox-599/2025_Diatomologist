package frc.robot.subsystems.elevator;

import frc.robot.Constants;

public final class ElevatorConstants {
  /* CHANGE LATER */

  public static enum ElevatorStates {
    CORALL1(0),
    CORALL2(1),
    CORALL3(2),
    CORALL4(3),
    STOW(4),
    CORALSTATIONINTAKE(5),
    ALGAEGROUNDINTAKE(6),
    ALGAEL2(7),
    ALGAEL3(8),
    BARGENET(9),
    PREP(10),
    PROCESSOR(11);

    private final int index;

    ElevatorStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  // Setpoint positions in inches
  public static final double[] heights = {
    14, // CORAL L1
    16, // CORAL L2
    33, // CORAL L3
    60, // CORAL L4
    5.5, // STOW
    0, // CORAL STATION INTAKE
    7, // ALGAE GROUND INTAKE
    27.5, // ALGAE L2
    42, // ALGAE L3
    92, // ALGAE SCORE
    12.0, // PREP
    1.0 // PROCESSOR
  };

  public static final int leaderMotorID = 13;
  public static final String leaderMotorCANbus = "rio";
  public static final int followerMotorID = 14;
  public static final String followerMotorCANbus = "rio";
  public static final boolean followerInverted = true;

  public static final double gearRatio = 5.4;
  public static final double inchesPerRev = 1.435406698564593;

  /* Position Setpoints (in inches) */
  public static final double elevatorLowerLimit = 0.0;
  public static final double elevatorUpperLimit = 89.0;
  public static final double homePositionOffset = 1.0;

  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV =
      Constants.kMotors
          .kKrakenX60Foc
          .kV; // there is no sensor to mechanism ratio so kV is the same as the motor's kV
  // if there is a sensor to mechanism ratio, kV = kV * sensor to mechanism ratio
  public static final double kS = 0.08;
  public static final double kG = 0.4;
  public static final double maxVelocityRotsPerSec = (12.0 - kS - kG) / kV * 0.85;
  public static final double maxAccelerationRotationsPerSecSQ = 2 * maxVelocityRotsPerSec * 0.85;

  // Add these for better PID tuning
  public static final double simkP = 8;
  public static final double simkI = 0.005;
  public static final double simkD = 0.0;
  public static final double simkF = 0.0;

  public static final double supplyCurrentLimitAmps = 40;
  public static final double statorCurrentLimitAmps = 90;

  public static final double positionToleranceInches = 0.5;

  public static final double elevatorMOI = 0.015;
}
