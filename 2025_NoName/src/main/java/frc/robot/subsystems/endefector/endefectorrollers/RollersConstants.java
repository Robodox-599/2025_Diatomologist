package frc.robot.subsystems.endefector.endefectorrollers;

public class RollersConstants {
  public static enum EndefectorRollerStates {
    SCORECORAL(0),
    SCOREALGAE(1),
    CORALSTATIONINTAKE(2),
    ADJUSTCORALAFTERSTATIONINTAKE(3),
    ALGAEINTAKE(4),
    HOLDALGAE(5),
    STOP(6),
    EJECT(7);

    private final int index;

    EndefectorRollerStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public static final double[] velocitys = {
    -0.65, // score coral
    -0.1, // score algae
    -0.15, // coral station intake
    0.0, // VELOCITY NOT USED - adjust coral after station intake
    0.6, // algae intake
    0.0, // VELOCITY NOT USED - hold algae
    0.0, // stop
    -0.8 // eject
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
  public static final double algaeDebounce = 0.5;

  public static final double rotationsToMoveAfterDetectingCoral = 0.0;
}
