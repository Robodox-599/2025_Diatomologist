package frc.robot.subsystems.drive.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class SimConstants {
  public static final double drive_kp = 0.1;
  public static final double drive_kd = 0;
  public static final double drive_ks = 0.05;
  public static final double drive_kv_rot = 0.91035;
  public static final double drive_kv = 1.95;
  public static final double turn_kp = 60.0;
  public static final double turn_kd = 0.001;
  public static final DCMotor drive_gearbox = DCMotor.getKrakenX60Foc(1);
  public static final DCMotor turn_gearbox = DCMotor.getKrakenX60Foc(1);
}
