package frc.robot.subsystems.rollers;

public class RollerConstants {
  public static final int motorID = 13;
  public static final String motorCANBus = "rio";

  public static final double simkP = 1.0;
  public static final double simkI = 0.0;
  public static final double simkD = 0.0;
  public static final double simkS = 0.0;
  public static final double simkV = 0.0;

  public static final double realkP = 0.1;
  public static final double realkI = 0.0;
  public static final double realkD = 0.0;
  public static final double realkS = 0.0;
  public static final double realkV = 0.0;

  public static final boolean EnableSupplyurrentLimit = true;
  public static final int LowerSupplyCurrentLimit = 50;
  public static final int PeakSupplyCurrentLimit = 50;
  public static final double LowerSupplyCurrentDuration = 0.1;

  public static final double kRollerSpeed = 0.35;
  public static final double kRollerBackSpeed = -0.35;

  public static final double kRollerInertia = 0.0;
  public static final double kRollerMotorGearRatio = 0.0;
}
