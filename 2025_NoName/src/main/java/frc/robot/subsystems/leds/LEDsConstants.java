package frc.robot.subsystems.leds;

public class LEDsConstants {
    public static enum LEDAnim {
        YesNote(0),
        NoNote(1),
        CanShoot(2),
        NoState(3);
        private final int index;

        LEDAnim(int index) {this.index = index;}

        public int getIndex() {return index;}
    }
    public static final int canID = 0;
    public static final String CANbus = "dingus";
    public static final int LEDS_PER_ANIMATION = 30;
}
