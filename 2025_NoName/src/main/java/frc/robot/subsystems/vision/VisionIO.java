package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public abstract class VisionIO {
  /**
   * Vision system constants, including the camera name, transform from the robot to the camera, and
   * a standard deviation factor for measurements.
   */
  protected VisionConstants constants =
      new VisionConstants("Default", new Transform3d(new Translation3d(), new Rotation3d()), 1.0);

  /** Indicates whether the camera is currently connected. */
  protected boolean cameraConnected = false;

  /** Stores the latest observed angles of a target as a pair of 2D rotations. */
  protected ObservedTargetRotations latestTargetAngle =
      new ObservedTargetRotations(new Rotation2d(), new Rotation2d());

  /**
   * An array containing pose observations made by the camera. Each observation includes metadata
   * such as timestamp, observed pose, ambiguity, and tag information.
   */
  protected PoseObservation[] poseObservations = new PoseObservation[0];

  /** The number of targets currently detected by the vision system. */
  protected double numTargets = 0;

  /** An array of IDs corresponding to the AprilTags detected by the vision system. */
  protected int[] aprilTagIds = new int[0];

  /**
   * Represents an observed pose of a target as a pair of 2D rotations (targetX, targetY). This is
   * used for tracking the orientation or position of a detected target.
   *
   * @param targetX the angle around the X-axis
   * @param targetY the angle around the Y-axis
   */
  public static record ObservedTargetRotations(Rotation2d targetX, Rotation2d targetY) {
    public Rotation2d getTargetX() {
      return targetX;
    }

    public Rotation2d getTargetY() {
      return targetY;
    }
  }

  /**
   * Represents a single pose observation from the camera. Includes information about the observed
   * pose and additional metadata.
   *
   * @param timestamp the time the pose was observed
   * @param observedPose the 3D pose detected by the camera
   * @param ambiguity a measure of how uncertain the observation is
   * @param tagCount the number of tags detected contributing to this pose
   * @param averageTagDistance the average distance to the detected tags
   */
  public static record PoseObservation(
      double timestamp,
      Pose3d observedPose,
      double ambiguity,
      int tagCount,
      double averageTagDistance) {
    public double getAverageTagDistance() {
      return averageTagDistance;
    }

    public int getTagCount() {
      return tagCount;
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
  }

  /**
   * @return Name of camera instance applied to the camera (grabbed from VisionConsants)
   */
  public String getName() {
    return "";
  }
  ;

  /** Updates Inputs to the given values from the IO layers */
  public void updateInputs() {}

  /**
   * @return Return VisionConstants of Camera.
   */
  public VisionConstants getVisionConstants() {
    return new VisionConstants(getName(), null, numTargets);
  }
  ;

  public int[] getAprilTagIds() {
    return aprilTagIds;
  }

  public PoseObservation[] getPoseObservation() {
    return poseObservations;
  }

  public ObservedTargetRotations getLatestTargetAngle() {
    return latestTargetAngle;
  }

  public double getNumTargets() {
    return numTargets;
  }

  public boolean getConnected() {
    return cameraConnected;
  }
}
