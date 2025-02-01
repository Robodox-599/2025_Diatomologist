package frc.robot.subsystems.algaegroundintake.intakeRollers;

public class IntakeRollersConstants {
    public static enum States {
        NOTDEPLOYED(0),
        STOW(1), 
        DEPLOYED(2),
        REVERSED(3),
        IDLE(4);

        private final int index;

        States(int index) {
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
}
