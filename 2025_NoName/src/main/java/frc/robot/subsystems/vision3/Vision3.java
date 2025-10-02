// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision3;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;

public class Vision3 {
  private final Vision3IOReal[] cameras;
  private final VisionConsumer consumer;

  public Vision3(VisionConsumer consumer, Vision3IOReal... cameras) {
    this.consumer = consumer;
    this.cameras = cameras;
  }

  // Functional interface for vision consumer
  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public record VisionUpdate(
      Pose2d pose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {

    private static final VisionUpdate kEmpty =
        new VisionUpdate(Pose2d.kZero, 0.0, Vision3Constants.kSingleTagStdDevs);

    public static VisionUpdate empty() {
      return kEmpty;
    }
  }

  public void updateInputs() {
    for (Vision3IOReal camera : cameras) {
      List<VisionUpdate> visionUpdates = camera.update();
      for (VisionUpdate update : visionUpdates) {
        if (!update.equals(VisionUpdate.empty())) {
          consumer.accept(update.pose, update.timestamp, update.visionMeasurementStdDevs);
        }
      }
    }
  }
}
