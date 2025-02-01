package frc.robot.subsystems.endefector.rollers;

public class RollersConstants {
    public static final int rollersMotorID = 0;
    public static final String rollersMotorCANBus = "rio";

    public static final boolean EnableCurrentLimit = true;
    public static final int ContinousCurrentLimit = 50;
    public static final int PeakCurrentLimit = 50;
    public static final double PeakCurrentDuration = 0.1;
  
    public static final double gearRatio = 1.0;
    public static final double rollersMOI = 0.04;

    public static final double simkP = 0.0;
    public static final double simkI = 0.0;
    public static final double simkD = 0.0;
    public static final double simkS = 0.0;
    public static final double simkV = 0.0;

    public static final double realP = 0.0;
    public static final double realI = 0.0;
    public static final double realD = 0.0;
    public static final double realS = 0.0;
    public static final double realV = 0.0;

    public static final double nominalVoltage = 12.0;

    public static final int CANrangeId = 0;
    public static final String CANrangeCANbus = "rio";
    public static final double rangeTolerance = 0.0;
}
