package frc.robot.subsystems.endefector.endefectorrollers;

public class RollersConstants {
  public static enum EndefectorRollerStates {
    INTAKING_CORAL_STATION(0),
    ENSURING_CORAL(1),
    INTAKING_ALGAE(2),
    HOLD_CORAL(3),
    HOLD_ALGAE(4),
    SCORING_CORAL_TROUGH(5),
    SCORING_CORAL_BRANCH(6),
    SCORING_ALGAE(7),
    STOPPED(8);

    private final int index;

    EndefectorRollerStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public static final double[] rollersVelocities = {
    -0.15, // intaking coral station
    0.04, // ensuring coral
    0.6, // intaking algae
    0.0, // holding coral
    0.0, // VELOCITY NOT USED - holding algae
    0.25, // scoring coral trough
    -0.4, // scoring coral branch
    -0.6, // scoring algae
    0.0, // stopped
  };

  public static final int rollersMotorID = 16;
  public static final String rollersMotorCANBus = "rio";

  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  public static final double gearRatio = 1.5;
  public static final double rollersMOI = 0.04;

  public static final double simkP = 7.0;
  public static final double simkI = 0.0;
  public static final double simkD = 0.0;
  public static final double simkS = 0.0;
  public static final double simkV = 0.0;

  public static final double realP = 0.0;
  public static final double realI = 0.0;
  public static final double realD = 0.0;
  public static final double realS = 0.0;
  public static final double realV = 0.0;

  public static final double rollersDutyCycleOutHoldAlgae = 0.2;

  public static final int beakBreakPort = 3;

  public static final double beamBreakDebounce = 0.3;
  public static final double ensureCoralDebounce = 0.6;
  public static final double algaeDebounce = 0.2;
  public static final double coralL1Debounce = 1;

  public static final double rotationsToMoveAfterDetectingCoral = 0.0;
}
