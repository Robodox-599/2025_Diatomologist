package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import frc.robot.Constants;

public final class ElevatorConstants {
  /* CHANGE LATER */

  public static enum ElevatorStates {
    L1(0),
    L2(1),
    L3(2),
    L4(3),
    STOW(4),
    INTAKE(5),
    GROUNDINTAKE(6),
    ALGAE_L2(7),
    ALGAE_L3(8),
    PREP(9);

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
    0.0, // L1
    20.0, // L2
    40.0, // L3
    60.0, // L4
    5.5, // STOW
    10, // STATION INTAKE
    15, // ALGAE GROUND INTAKE
    15, // ALGAE L2
    35, // ALGAE L3
    10 // PREP
  };

  public static final int leaderMotorID = 13;
  public static final String leaderMotorCANbus = "rio";
  public static final int followerMotorID = 14;
  public static final String followerMotorCANbus = "rio";
  public static final boolean followerInverted = true;

  public static final int limitSwitchDioPort = 10;
  public static final double gearRatio = 5.4;
  public static final double drumCircumferenceInches = Math.PI * 2.0;
  public static final double inchesPerRev =
      drumCircumferenceInches / gearRatio; // reduction so dividing by gear ratio
  public static final double drumRadiusMeters = Units.Inches.of(2).magnitude();

  /* Position Setpoints (in inches) */
  public static final double elevatorLowerLimit = 0.0;
  public static final double elevatorUpperLimit = 89.0;
  public static final double homePositionOffset = 1.0;

  public static final double kP = 1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV =
      Constants.kMotors
          .kKrakenX60Foc
          .kV; // there is no sensor to mechanism ratio so kV is the same as the motor's kV
  // if there is a sensor to mechanism ratio, kV = kV * sensor to mechanism ratio
  public static final double kS = 0.0;
  public static final double kG = 0.0;
  public static final double maxVelocityRotsPerSec = (12.0 - kS - kG) / kV;
  public static final double maxAccelerationRotationsPerSecSQ = 120.0;

  // Add these for better PID tuning
  public static final double simkP = 8;
  public static final double simkI = 0.005;
  public static final double simkD = 0.0;
  public static final double simkF = 0.0;

  public static final double supplyCurrentLimitAmps = 40;
  public static final double statorCurrentLimitAmps = 0.1;

  public static final double positionToleranceInches = 0.5;
  public static final double velocityToleranceInchesPerSecond = 0.5;

  public static final double elevatorMOI = 0.015;
}
