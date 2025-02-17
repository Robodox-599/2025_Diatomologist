package frc.robot.subsystems.leds;

public class LEDsConstants {
    public static enum LEDAnim {
        StationIntake,
        AlgaeIntake,
        NoState,
        Scored,
        Climb,
        AutoAlign,
        ReadyToScore;
    }
    public static final int canID = 21;
    public static final String CANbus = "rio";
    public static final int LEDS_PER_ANIMATION = 30;
}
