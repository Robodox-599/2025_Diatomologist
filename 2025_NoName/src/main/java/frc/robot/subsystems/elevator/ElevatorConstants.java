package frc.robot.subsystems.elevator;

import frc.robot.Constants;

public final class ElevatorConstants {
  /* CHANGE LATER */

  public static enum ElevatorStates {
    INTAKING_CORAL_STATION(0),
    INTAKING_ALGAE_GROUND(1),
    INTAKING_ALGAE_LOLLIPOP(2),
    POSITION_ALGAE_L2(3),
    POSITION_ALGAE_L3(4),
    POSITION_PREPARED(5),
    POSITION_CORAL_L1(6),
    POSITION_CORAL_L2(7),
    POSITION_CORAL_L3(8),
    POSITION_CORAL_L4(9),
    POSITION_CORAL_L4_AUTO(10),
    POSITION_ALGAE_PROCESSOR(11),
    POSITION_ALGAE_BARGE(12);

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
    0.0, // INTAKING ALGAE GROUND
    7.0, // INTAKING ALGAE LOLLIPOP
    22.0, // INTAKING ALGAE L2 // 27.5
    36.5, // INTAKING ALGAE L3 // 42
    12.0, // POSITION PREPARED
    14.0, // POSITION CORAL L1 // 14
    34.5, // POSITION CORAL L2 // 16
    50.5, // POSITION CORAL L3 // 33
    62.0, // POSITION CORAL L4 // 60
    62.0, // POSITION CORAL L4 AUTO
    5.0, // POSITION ALGAE PROCESSOR
    85.0, // POSITION ALGAE BARGE
  };

  public static final int leaderMotorID = 13;
  public static final String leaderMotorCANbus = "rio";
  public static final int followerMotorID = 14;
  public static final String followerMotorCANbus = "rio";
  public static final boolean followerInverted = true;

  public static final double gearRatio = 6.0;
  public static final double inchesPerRev = 1.073;

  /* Position Setpoints (in inches) */
  public static final double elevatorHardLowerLimit = 0.0;
  public static final double elevatorHardUpperLimit = 85.0;
  public static final double homePositionOffset = 1.0;

  public static final double kP = 1.5;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV =
      Constants.kMotors
          .kKrakenX60Foc
          .kV; // there is no sensor to mechanism ratio so kV is the same as the motor's kV
  // if there is a sensor to mechanism ratio, kV = kV * sensor to mechanism ratio
  public static final double kS = 0.135;
  public static final double kG = 0.365;
  public static final double maxVelocityRotsPerSec = (12.0 - kS - kG) / kV;
  public static final double maxAccelerationRotationsPerSecSQ =
      maxVelocityRotsPerSec
          / 0.15; // set update to 150hz, set max accel to 15 * velocity, find the max accel, and
  // multiply by 0.8

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
