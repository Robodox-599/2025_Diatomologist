package frc.robot.subsystems.endefector.endefectorrollers;

public class RollersConstants {
  public static enum EndefectorRollerStates {
    INTAKING_CORAL_STATION(0),
    INTAKING_ALGAE(1),
    HOLD_CORAL(2),
    HOLD_ALGAE(3),
    SCORING_CORAL(4),
    SCORING_ALGAE(5),
    STOPPED(6);

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
    0.6, // intaking algae
    0.0, // holding coral
    0.0, // VELOCITY NOT USED - holding algae
    -0.4, // scoring coral
    -0.2, // scoring algae
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
  public static final double ensureCoralDebounce = 0.8;
  public static final double algaeDebounce = 0.5;

  public static final double rotationsToMoveAfterDetectingCoral = 0.0;
}
