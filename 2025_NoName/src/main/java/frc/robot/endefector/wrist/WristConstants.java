package frc.robot.subsystems.endefector.wrist;

public class WristConstants {
    public static final int wristMotorID = 0;
    public static final String wristMotorCANBus = "rio";

    public static final boolean EnableCurrentLimit = true;
    public static final int ContinousCurrentLimit = 50;
    public static final int PeakCurrentLimit = 50;
    public static final double PeakCurrentDuration = 0.1;
  
    public static final double gearRatio = 0.0;
    public static final double wristMOI = 0.04;

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

    public static final double simP = 0.0;
    public static final double simI = 0.0;
    public static final double simD = 0.0;
    public static final double simV = 0.0;
    public static final double simS = 0.0;
  
    public static final double simVelocityConstant = 0.2;

    public static final double wristExtendsSlot = 0.0;
    public static final double wristRetractSlot = 1.0;
}
