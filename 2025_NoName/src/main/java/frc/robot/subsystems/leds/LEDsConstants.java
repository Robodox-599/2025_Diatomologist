package frc.robot.subsystems.leds;

public class LEDsConstants {
  public static enum LEDAnim {
    NoState,
    Intaking,
    NoGamePiece,
    YesGamePiece,
    CanScore,
    Climb;
  }

  public static final int canID = 0;
  public static final String CANbus = "dingus";
  public static final int LEDS_PER_ANIMATION = 30;
}
