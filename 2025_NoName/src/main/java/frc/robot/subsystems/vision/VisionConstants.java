package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Represents constants and parameters used for vision This record defines the camera configuration,
 * its transform relative to the robot, and standard deviation factors for pose estimation.
 *
 * @param cameraName the name or identifier of the camera
 * @param robotToCameraTransform3d the 3D transform from the robot's coordinate frame to the
 *     camera's
 * @param cameraStdDevFactor a scaling factor applied to the standard deviation of the camera
 *     measurements. If value is 1, then no factor will be applied to the standard deviation of the
 *     camera measurements
 */
public record VisionConstants(
    String cameraName, Transform3d robotToCameraTransform3d, double cameraStdDevFactor) {

  /**
   * The maximum non-Filtered ambiguity in a pose observation. Observations with ambiguity higher
   * than this threshold will be filtered out.
   */
  public double maxAmbiguity() {
    return 0.3;
  }

  /**
   * The maximum allowed Z-axis error in a pose observation (in meters). Observations greater than
   * this error will be rejected.
   */
  public double getMaxZError() {
    return 0.2;
  }

  public double getMaxAngleError() {
    return 0.08726646259971647; // 5 degrees in radians, used for filtering out bad observations
  }

  /**
   * The baseline standard deviation for linear measurements at 1 meter distance and 1 detected tag.
   * This value is adjusted dynamically (based on actual distance and the number of detected tags),
   * in the Vision class.
   */
  public double linearStdDevBaseline() {
    return 0.02;
  }

  /**
   * The baseline standard deviation for angular measurements at 1 meter distance and 1 detected
   * tag. This value is adjusted dynamically (based on actual distance and the number of detected
   * tags), in the Vision class.
   */
  public double angularStdDevBaseline() {
    return 0.06;
  }
}
