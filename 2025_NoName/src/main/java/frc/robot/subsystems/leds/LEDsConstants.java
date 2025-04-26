package frc.robot.subsystems.leds;

public class LEDsConstants {
  public static enum LEDStates {
    CORALSTATIONINTAKE(0),
    ALGAEINTAKE(1),
    INTAKED(2),
    SCORING(3),
    SCORED(4),
    AUTOALIGN(5),
    PREPARED(6),
    OVERRIDE(7),
    IDLE(8);

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
