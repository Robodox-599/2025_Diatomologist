package frc.robot.subsystems.elevator;

import frc.robot.Constants;

public final class ElevatorConstants {
  /* CHANGE LATER */

  public static enum ElevatorStates {
    INTAKING_CORAL_STATION(0),
    INTAKING_ALGAE_GROUND(1),
    INTAKING_ALGAE_L2(2),
    INTAKING_ALGAE_L3(3),
    POSITION_PREPARED(4),
    POSITION_CORAL_L1(5),
    POSITION_CORAL_L2(6),
    POSITION_CORAL_L3(7),
    POSITION_CORAL_L4(8),
    POSITION_ALGAE_PROCESSOR(9),
    POSITION_ALGAE_BARGE(10);

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
    0.0, // INTAKING CORAL STATION
    7.0, // INTAKING ALGAE GROUND
    28.5, // INTAKING ALGAE L2 // 27.5
    42.0, // INTAKING ALGAE L3 // 42
    12.0, // POSITION PREPARED
    12.0, // POSITION CORAL L1 // 14
    18.0, // POSITION CORAL L2 // 16
    37.0, // POSITION CORAL L3 // 33
    62.0, // POSITION CORAL L4 // 60
    1.0, // POSITION ALGAE PROCESSOR
    92, // POSITION ALGAE BARGE
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
  public static final double maxVelocityRotsPerSec = (12.0 - kS - kG) / kV * 1.5;
  public static final double maxAccelerationRotationsPerSecSQ = 2 * maxVelocityRotsPerSec * 1.5;

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
