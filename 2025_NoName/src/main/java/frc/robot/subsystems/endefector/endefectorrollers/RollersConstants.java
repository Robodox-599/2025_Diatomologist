package frc.robot.subsystems.endefector.endefectorrollers;

import frc.robot.Constants;
import frc.robot.util.SubsystemUtil;

public class RollersConstants {
  public static enum EndefectorRollerStates {
    INTAKING_CORAL_STATION(0),
    ENSURING_CORAL_FORWARDS(1),
    ENSURING_CORAL_BACKWARDS(2),
    INTAKING_ALGAE(3),
    HOLD_CORAL(4),
    SCORING_CORAL_TROUGH(5),
    SCORING_CORAL_L2_L3(6),
    SCORING_CORAL_L4(7),
    SCORING_ALGAE(8),
    STOPPED(9);

    private final int index;

    EndefectorRollerStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public static final double[] endefectorRollersVelocities = {
    0.4, // intaking coral station
    0.25, // ensuring coral forwards
    -0.1, // ensuring coral backwards
    -0.6, // intaking algae
    0.0, // holding coral
    -0.6, // scoring coral trough
    -0.6, // scoring coral l2 or l3
    1.0, // scoring coral l4
    0.3, // scoring algae
    0.0, // stopped
  };

  public static final int endefectorRollersMotorID = 16;
  public static final String endefectorRollersMotorCANBus = "rio";

  public static final int rampRollersMotorID = 28;
  public static final String rampRollersMotorCANBus = "rio";

  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  public static final double endefectorGearRatio = 6;
  public static final double rampGearRatio = 2;
  public static final double rollersMOI = 0.04;

  public static final double simkP = 7.0;
  public static final double simkI = 0.0;
  public static final double simkD = 0.0;
  public static final double simkS = 0.0;
  public static final double simkV = 0.0;

  public static final double realP = 3.0;
  public static final double realI = 0.0;
  public static final double realD = 0.0;
  public static final double realkS = 0.02;
  public static final double realEndefectorkV =
      Constants.kMotors.kKrakenX60Foc.kV * endefectorGearRatio;
  public static final double realRampkV = Constants.kMotors.kKrakenX60Foc.kV * rampGearRatio;

  public static final double rollersDutyCycleOutHoldAlgae = -0.1;
  public static final double algaeStallStatorCurrentAmps = 20;
  public static final double rampRollersVelocitySetpoint =
      SubsystemUtil.endefectorRollersStateToVelocity(EndefectorRollerStates.INTAKING_CORAL_STATION)
          * (rampGearRatio / endefectorGearRatio); // account for gear ratio

  public static final int rampBeamBreakPort = 1;
  public static final int transitionBeamBreakPort = 3;
  public static final int endefectorBeamBreakPort = 2;

  public static final double rampCoralDebounce = 0.1;
  public static final double coralIntakeDebounce = 0.05;
  public static final double algaeGroundIntakeDebounce = 0.15;
  public static final double algaeReefIntakeDebounce = 0.3;
  public static final double coralTroughScoreDebounce = 0.3;
  public static final double coralBranchScoreDebounce = 0.1;
  public static final double algaeScoreDebounce = 1.0;
}
