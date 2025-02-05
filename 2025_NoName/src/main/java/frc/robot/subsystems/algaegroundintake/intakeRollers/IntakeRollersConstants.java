package frc.robot.subsystems.algaegroundintake.intakeRollers;

public class IntakeRollersConstants {
  public static enum AlgaeRollerStates {
    STOW(1),
    INTAKE(2),
    REVERSE(3);

    private final int index;

    AlgaeRollerStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public static final int rollersMotorID = 1;
  public static final String rollersMotorCANBus = "rio";

  public static final double gearRatio = 0;
  public static final double rollersMOI = 0;

  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;

  public static final double simVelocityConstant = 0.2;
  public static final double realP = 0.0;
  public static final double realI = 0.0;
  public static final double realD = 0.0;
  public static final double realS = 0.0;
  public static final double realV = 0.0;

  public static final double simkP = 0.0;
  public static final double simkI = 0.0;
  public static final double simkD = 0.0;
  public static final double simkS = 0.0;
  public static final double simkV = 0.0;

  public static final double rollersReverseVelocity = 0.0;

  public static final double[] velocitys = {
    0.0, // stop
    0.0, // score
    0.0, // intake
  };
}
