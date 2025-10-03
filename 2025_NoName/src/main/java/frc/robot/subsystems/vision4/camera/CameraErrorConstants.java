package frc.robot.subsystems.vision4.camera;

public class CameraErrorConstants {
  public static final double maxZError = 0.2;
  public static final double maxAngleError =
      0.08726646259971647; // 5 degrees in radians, used for filtering out bad observations
  public static final double maxAmbiguity = 0.25;
}
