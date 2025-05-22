package frc.robot.subsystems.leds;

public class LEDsConstants {
  public static enum LEDStates {
    INTAKING_CORAL_STATION(0),
    INTAKING_ALGAE_GROUND(1),
    INTAKING_ALGAE_L2(2),
    INTAKING_ALGAE_L3(3),
    PREPARED(4),
    SCORING_CORAL_L1(5),
    SCORING_CORAL_L2(6),
    SCORING_CORAL_L3(7),
    SCORING_CORAL_L4(8),
    SCORING_ALGAE_PROCESSOR(9),
    SCORING_ALGAE_BARGE(10),
    NO_STATE(11);

    private final int index;

    LEDStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  // {r, g, b, w, speed}
  public static final double[][] colors = {
    {255, 255, 255, 100, 0.30}, // CORAL STATION INTAKE - WHITE
    {0, 255, 255, 100, 0.30}, // ALGAE INTAKE - CYAN
    {0, 255, 0, 100, 0.5}, // INTAKED - GREEN
    {255, 0, 0, 100, 0.30}, // SCORING - RED
    {255, 255, 0, 100, 0.50}, // SCORED - YELLOW
    {255, 82, 0, 0, 0.50}, // AUTO ALIGN - DIFFERENT ANIMATE USED
    {0, 0, 255, 100, 0.55}, // PREPARED
    {130, 0, 0, 50, 0.5}, // OVERRIDE
    {255, 82, 0, 0, 0.50}, // IDLE
  };

  public static final int canID = 21;
  public static final String CANbus = "rio";
  public static final int LEDS_PER_ANIMATION = 52;
}
