package frc.robot.subsystems.endefector.endefectorwrist;

public class WristConstants {
  // motor info
  public static final int wristMotorID = 15;
  public static final int cancoderID = 17;
  public static final String wristMotorCANBus = "rio";
  public static final double gearRatio = 58.78;
  public static final double wristMOI = 0.04;
  public static final double wristPositionTolerance = 0.03; // rotations

  // current limit stuff
  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  public static final double realkP = 15.0;
  public static final double realkI = 0.0;
  public static final double realkD = 0.4;
  public static final double realkS = -0.1;
  public static final double realkV = 6.96078949;
  public static final double realkG = -0.39;

  // sim stuff
  public static final double simkP = 6.9;
  public static final double simkI = 0.5;
  public static final double simkD = 2.25;
  public static final double simkV = 0.0;
  public static final double simkS = 0.0;
  public static final double simVelocityConstant = 0.2;

  public static final double cancoderOffset = -0.032958984375;

  // setpoints
  public static final double wristMinAngle = 0.52;
  public static final double wristMaxAngle = 1.05;

  // wrist state stuff
  public static enum WristStates {
    STOW(0),
    PREPARE(1),
    SCORING(2),
    OVERRIDE(3),
    REEFINTAKE(4),
    GROUNDINTAKE(5),
    STATIONINTAKE(6),
    CLIMB(7);
    private final int index;

    WristStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public static final double[] setpoints = {
    0.52, // stow
    0.75, // PREPARE
    0.68, // SCORING
    0.52, // OVERRIDE
    0.827, // reef intake
    0.879, // ground intake
    0.64, // station intake
    0.52, // climb,
  };
}
