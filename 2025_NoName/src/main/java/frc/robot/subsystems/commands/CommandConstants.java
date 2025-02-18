package frc.robot.subsystems.commands;

public class CommandConstants {
  public static final double deadband = 0.1;
  public static final double angle_kp = 5.0;
  public static final double angle_kd = 0;
  public static final double angle_max_velocity = 8.0;
  public static final double angle_max_acceleration = 20.0;
  public static final double ff_start_delay = 2.0; // Secs
  public static final double ff_ramp_rate = 0.1; // Volts/Sec
  public static final double wheel_radius_max_velocity = 0.25; // Rad/Sec
  public static final double wheel_radius_ramp_rate = 0.05; // Rad/Sec^2
}
