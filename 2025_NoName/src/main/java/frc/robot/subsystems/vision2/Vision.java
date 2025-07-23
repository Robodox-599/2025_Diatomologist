package frc.robot.subsystems.vision2;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.FieldConstants;

public class Vision {
  private final CameraReal[] cameras;
  private final VisionConsumer consumer;

  public Vision(VisionConsumer consumer, CameraReal... cameras) {
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

  private boolean checkPose(PoseObservation observation, int cameraIndex) {
    Pose3d pose = observation.getObservedPose();
    Translation2d simplePose = pose.getTranslation().toTranslation2d();
    return (simplePose.getX() < 0.0
        || simplePose.getX() > FieldConstants.fieldLength
        || simplePose.getY() < 0.0
        || simplePose.getY() > FieldConstants.fieldWidth
        || Double.isNaN(simplePose.getX())
        || Double.isNaN(simplePose.getY())
        || Math.abs(pose.getTranslation().getZ())
            > cameras[cameraIndex].getConstants().getMaxZError()
        || pose.getRotation().getY() > cameras[cameraIndex].getConstants().getMaxAngleError()
        || pose.getRotation().getX() > cameras[cameraIndex].getConstants().getMaxAngleError()
        || observation.getAverageTagDistance() > 5.5);
  }

  public void update() {
    for (int i = 0; i < cameras.length; i++) {
      CameraReal camera = cameras[i];
      PoseObservation[] observations = camera.update();
      DogLog.log(
          "Vision/" + camera.getConstants().cameraName() + "/Observations", observations.length);

      if (observations.length == 0) {
        continue;
      }

      for (PoseObservation observation : observations) {
        if (checkPose(observation, i)) {
          DogLog.log(
              "Vision/" + camera.getConstants().cameraName() + "/RejectedRobotPose",
              observation.getObservedPose());
          continue;
        }

        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.getTagCount();

        double linearStdDev = camera.getConstants().linearStdDevBaseline() * stdDevFactor;
        double angularStdDev = camera.getConstants().angularStdDevBaseline() * stdDevFactor;

        linearStdDev *= camera.getConstants().cameraStdDevFactor();
        angularStdDev *= camera.getConstants().cameraStdDevFactor();

        // Check whether to reject pose

        DogLog.log("Vision/" + camera.getConstants().cameraName() + "/PoseAccepted?", true);
        DogLog.log(
            "Vision/" + camera.getConstants().cameraName() + "/RobotPose",
            observation.getObservedPose());

        consumer.accept(
            observation.getObservedPose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }
    }
  }
}
