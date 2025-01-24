package frc.robot.util;

import dev.doglog.DogLog;
import frc.robot.subsystems.vision.VisionIO;

public class VisionLog extends DogLog {
  private static String key;

  public static void log(VisionIO io) {
    key = "Vision" + "/" + io.getName();

    log(key + "/CameraConnected", io.getConnected());

    // Latest target information
    if (io.getLatestTargetAngle() != null) {
      log(key + "/LatestTarget/Yaw", io.getLatestTargetAngle().getTargetX().getDegrees());
      log(key + "/LatestTarget/Pitch", io.getLatestTargetAngle().getTargetY().getDegrees());
    }

    // Number of detected targets
    log(key + "/NumTargets", io.getNumTargets());

    // Pose observations
    if (io.getPoseObservation() != null && io.getPoseObservation().length > 0) {
      // Log the most recent pose observation
      var latestPose = io.getPoseObservation()[io.getPoseObservation().length - 1];
      log(key + "/LatestPose/Timestamp", latestPose.getTimestamp());
      log(key + "/LatestPose/Position/X", latestPose.getObservedPose().getX());
      log(key + "/LatestPose/Position/Y", latestPose.getObservedPose().getY());
      log(key + "/LatestPose/Position/Z", latestPose.getObservedPose().getZ());
      log(key + "/LatestPose/Rotation/Roll", latestPose.getObservedPose().getRotation().getX());
      log(key + "/LatestPose/Rotation/Pitch", latestPose.getObservedPose().getRotation().getY());
      log(key + "/LatestPose/Rotation/Yaw", latestPose.getObservedPose().getRotation().getZ());
      log(key + "/LatestPose/Ambiguity", latestPose.getAmbiguity());
      log(key + "/LatestPose/TagCount", latestPose.getTagCount());
      log(key + "/LatestPose/AverageTagDistance", latestPose.getAverageTagDistance());
    }

    // April tag IDs
    if (io.getAprilTagIds() != null && io.getAprilTagIds().length > 0) {
      log(key + "/AprilTagCount", io.getAprilTagIds().length);
      // Log detected tag IDs as a comma-separated string
      StringBuilder tagList = new StringBuilder();
      for (int i = 0; i < io.getAprilTagIds().length; i++) {
        tagList.append(io.getAprilTagIds()[i]);
        if (i < io.getAprilTagIds().length - 1) {
          tagList.append(",");
        }
      }
      log(key + "/DetectedTagIds", tagList.toString());
    }
  }
}
