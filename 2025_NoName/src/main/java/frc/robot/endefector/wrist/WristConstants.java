package frc.robot.endefector.wrist;

public class WristConstants {
    //motor info
    public static final int wristMotorID = 0;
    public static final String wristMotorCANBus = "rio";
    public static final double gearRatio = 0.0;
    public static final double wristMOI = 0.04;

    //current limit stuff
    public static final boolean EnableCurrentLimit = true;
    public static final int ContinousCurrentLimit = 50;
    public static final int PeakCurrentLimit = 50;
    public static final double PeakCurrentDuration = 0.1;
  
    //real pid
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

    //sim stuff
    public static final double simkP = 0.0;
    public static final double simkI = 0.0;
    public static final double simkD = 0.0;
    public static final double simkV = 0.0;
    public static final double simkS = 0.0;
    public static final double simVelocityConstant = 0.2;


    public static final double wristExtendsSlot = 0.0;
    public static final double wristRetractSlot = 1.0;

    public static final int cancoderID = 0;
    public static final double cancoderOffset = 0.0;

    //setpoints
    public static final double wristLowerLimit = 0.0;
    public static final double wristUpperLimit = 0.0;
    public static final double homePositionOffset = 1.0;
    
    public static final double stowHeight = 0;
    public static final double groundIntakeHeight = 0;
    public static final double stationIntakeHeight = 0;
    public static final double scoringHeight = 0;

    public static final double inchesPerRev = 0;
    // public static final double climb = 0;


    //wrist state stuff
    public static enum WristStates {
        STOW(1),
        SCORING(2), 
        OVERRIDE(3), 
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
        0.0,    // stow
        0.0,   // scoring
        0.0,   // ground intake
        0.0,    // station intake
        0.0 // climb
    };
}