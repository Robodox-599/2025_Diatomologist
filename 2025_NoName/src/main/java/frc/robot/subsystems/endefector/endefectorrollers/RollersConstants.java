package frc.robot.subsystems.endefector.endefectorrollers;

public class RollersConstants {
  public static enum EndefectorRollerStates {
    SCORE(0),
    STOP(1),
    INTAKE(2),
    REEFINTAKE(3),
    ALGAEINTAKE(4);

    private final int index;

    EndefectorRollerStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public static final int rollersMotorID = 16;
  public static final int CANrangeId = 18;
  public static final String rollersMotorCANBus = "rio";
  public static final String CANrangeCANbus = "rio";

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

  public static final double rollersScoreSpeed = 0.2;

  public static final double centerOffset = 0.0;
  public static final double algaeIntakeStalling = 65;

  // This value will be the distance the canrage reads without coral in it, use inches.
  public static final double noCoralDistance = 11;
  // This value will be
  public static final double detectionDistance = noCoralDistance - 3.5;

  public static final double[] velocitys = {
    20.0, // score
    0.0, // stop
    40.0, // intake
    80,
    60.0 // reefintake
  };
}
