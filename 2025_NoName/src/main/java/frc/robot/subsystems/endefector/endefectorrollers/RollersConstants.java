package frc.robot.subsystems.endefector.endefectorrollers;

import frc.robot.Constants;

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

  public static final double[] rollersVelocities = {
    0.4, // intaking coral station
    0.4, // ensuring coral forwards
    -0.1, // ensuring coral backwards
    -1.0, // intaking algae
    0.0, // holding coral
    -0.5, // scoring coral trough
    -0.6, // scoring coral l2 or l3
    1.0, // scoring coral l4
    1.0, // scoring algae
    0.0, // stopped
  };

  public static final int rollersMotorID = 16;
  public static final String rollersMotorCANBus = "rio";

  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  public static final double gearRatio = 6;
  public static final double rollersMOI = 0.04;

  public static final double simkP = 7.0;
  public static final double simkI = 0.0;
  public static final double simkD = 0.0;
  public static final double simkS = 0.0;
  public static final double simkV = 0.0;

  public static final double realP = 0.0;
  public static final double realI = 0.0;
  public static final double realD = 0.0;
  public static final double realkS = 0.0;
  public static final double realkV = Constants.kMotors.kKrakenX60Foc.kV * gearRatio;

  public static final double rollersDutyCycleOutHoldAlgae = -0.1;
  public static final double algaeStallStatorCurrentAmps = 20;

  public static final int rampBeamBreakPort = 1;
  public static final int endefectorBeamBreakPort = 0;

  public static final double rampCoralDebounce = 0.1;
  public static final double coralIntakeDebounce = 0.05;
  public static final double algaeIntakeDebounce = 0.25;
  public static final double coralTroughScoreDebounce = 0.3;
  public static final double coralBranchScoreDebounce = 0.1;
  public static final double algaeScoreDebounce = 0.5;
}
