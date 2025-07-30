package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;

/**
 * Represents a single pose observation from the camera. Includes information about the observed
 * pose and additional metadata.
 *
 * @param timestamp the time the pose was observed
 * @param observedPose the 3D pose detected by the camera
 * @param ambiguity a measure of how uncertain the observation is
 * @param tagsList the list of detected tags contributing to this pose
 * @param averageTagDistance the average distance to the detected tags
 * @param tagArea the average tag area of the detected tags
 */
public record PoseObservation(
    double timestamp,
    Pose3d observedPose,
    double ambiguity,
    List<Integer> tagsList,
    double averageTagDistance,
    double tagArea) {
  public double getAverageTagDistance() {
    return averageTagDistance;
  }

  public int getTagCount() {
    return tagsList.size();
  }

  public double getAmbiguity() {
    return ambiguity;
  }

  public Pose3d getObservedPose() {
    return observedPose;
  }

  public double getTimestamp() {
    return timestamp;
  }

  public double getTagArea() {
    return tagArea;
  }
}
