package frc.robot.subsystems.endefector.endefectorrollers;

public class RollersConstants {
  public static enum EndefectorRollerStates {
    SCORECORAL(0),
    SCOREALGAE(1),
    CORALSTATIONINTAKE(2),
    ALGAEINTAKE(3),
    HOLDCORAL(4),
    HOLDCORALAFTERSTATIONINTAKE(5),
    HOLDALGAE(6),
    STOP(7),
    EJECT(8);

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

  public static final double rollersCoralScoreSpeed = 0.25;
  public static final double rollersCoralStationIntakeSpeed = 0.09;
  public static final double rollersEjectSpeed = rollersCoralScoreSpeed;
  public static final double rollersAlgaeIntakeSpeed = -rollersCoralScoreSpeed * 2;
  public static final double rollersAlgaeScoreSpeed = rollersCoralScoreSpeed * 2;

  public static final double rollersDutyCycleAlgaeHoldVoltage = -0.2;

  public static final double centerOffset = 0.0;
  public static final double algaeIntakeStalling = 65;

  // This value will be the distance the canrage reads without coral in it, use inches.
  public static final double noCoralDistance = 11;
  // This value will be
  public static final double detectionDistance = noCoralDistance - 3.5;

  public static final int beakBreakPort = 3;
  public static final int PESensorPort = 5;
  public static final double beamBreakDebounce = 0.0;

  public static final double distanceToMove = 0.0;

  // public static final double[] velocitys = {
  //   0.25, // score
  //   0.0, // stop
  //   0.05, // intake
  //   -0.25, // reef intake
  //   -0.25, // algae intake
  //   -0.25 // reverse
  // };
}
