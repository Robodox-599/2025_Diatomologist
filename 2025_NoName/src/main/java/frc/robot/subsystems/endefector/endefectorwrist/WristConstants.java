package frc.robot.subsystems.endefector.endefectorwrist;

public class WristConstants {
  // motor info
  public static final int wristMotorID = 0;
  public static final String wristMotorCANBus = "rio";
  public static final double gearRatio = 58.78;
  public static final double wristMOI = 0.04;
  public static final double wristPositionTolerance = 0.3; // degrees
  public static final double inchesPerRev = 10;

  // current limit stuff
  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  // real pid

  // .5 to mvoe up
  // .3 to move down
  // subtract the two (0.5-0.3 = 0.2) and divide the difference by 2, (0.2/2 = 0.1)
  // kS = 0.3 + (0.2/2)
  // kG = 0.2/2
  // goin up = kS+kG
  // goin down = kG-kS

  // max velocity on elevator is your kV times twelve
  // max acceleration on elevator is the time it takes to accelerate, and i take my velocity divided
  // by that time. boom max accel
  // my max velocity is my freespeed minus my speed loss, speed loss calculation =
  // (12+kS+kG)/freeSpeed
  //
  public static final double realExtendkP = 0.0;
  public static final double realExtendkI = 0.0;
  public static final double realExtendkD = 0.0;
  public static final double realExtendkS = 0.0;
  public static final double realExtendkV = 0.0;

  public static final double realRetractkP = 0.0;
  public static final double realRetractkI = 0.0;
  public static final double realRetractkD = 0.0;
  public static final double realRetractkS = 0.0;
  public static final double realRetractkV = 0.0;

  // sim stuff
  public static final double simkP = 6.9;
  public static final double simkI = 0.5;
  public static final double simkD = 2.25;
  public static final double simkV = 0.0;
  public static final double simkS = 0.0;
  public static final double simVelocityConstant = 0.2;

  public static final int wristExtendsSlot = 0;
  public static final int wristRetractSlot = 1;

  public static final int cancoderID = 0;
  public static final double cancoderOffset = 0.0;

  // setpoints
  public static final double wristMinAngle = 0.0;
  public static final double wristMaxAngle = 90.0;

  public static final double wristPositionLimit = 1;

  // wrist state stuff
  public static enum WristStates {
    STOW(0),
    SCORING(1),
    OVERRIDE(2),
    REEFINTAKE(3),
    GROUNDINTAKE(4),
    STATIONINTAKE(5),
    CLIMB(6);
    private final int index;

    WristStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public static final double[] setpoints = {
    10.0, 20.0, // stow
    83.316, // scoring
    83.6596, // reef intake
    47.5356, // ground intake
    64.5866, // station intake
    60.0, // climb,
    80.0
  };
}
