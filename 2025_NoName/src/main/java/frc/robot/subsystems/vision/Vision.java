package frc.robot.subsystems.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final Alert[] disconnectedAlerts;
  private final Supplier<Pose2d> poseSupplier;

  public Vision(VisionConsumer consumer, Supplier<Pose2d> poseSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.poseSupplier = poseSupplier;
    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      disconnectedAlerts[i] = new Alert(io[i].getName() + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return io[cameraIndex].latestTargetAngle.targetX();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(poseSupplier);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!io[cameraIndex].cameraConnected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : io[cameraIndex].tagIds) {
        var tagPose = VisionConstants.aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : io[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose = checkPose(observation, cameraIndex);
        // Add pose to log
        robotPoses.add(observation.observedPose());
        if (rejectPose) {
          robotPosesRejected.add(observation.observedPose());
          continue;
        } else {
          robotPosesAccepted.add(observation.observedPose());
        }
      }

      PoseObservation selectedPose =
          selectClosestPose(io[cameraIndex].poseObservations, poseSupplier.get());

      // Calculate standard deviations
      double stdDevFactor =
          Math.pow(selectedPose.averageTagDistance(), 2.0) / selectedPose.tagCount();
      double linearStdDev =
          io[cameraIndex].getVisionConstants().linearStdDevBaseline() * stdDevFactor;
      double angularStdDev =
          io[cameraIndex].getVisionConstants().angularStdDevBaseline() * stdDevFactor;

      linearStdDev *= io[cameraIndex].getVisionConstants().cameraStdDevFactor();
      angularStdDev *= io[cameraIndex].getVisionConstants().angularStdDevBaseline();

      consumer.accept(
          selectedPose.getObservedPose().toPose2d(),
          selectedPose.timestamp(),
          VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      // Log camera datadata
      DogLog.log(
          "Vision/" + io[cameraIndex].getName() + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      DogLog.log(
          "Vision/" + io[cameraIndex].getName() + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      DogLog.log(
          "Vision/" + Integer.toString(cameraIndex) + "/AcceptedRobotPoses",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      DogLog.log(
          "Vision/" + Integer.toString(cameraIndex) + "/RejectedRobotPoses",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }
    // Log summary data
    DogLog.log("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    DogLog.log(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    DogLog.log(
        "Vision/Summary/AcceptedRobotPoses",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    DogLog.log(
        "Vision/Summary/RejectedRobotPoses",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  private PoseObservation selectClosestPose(
      PoseObservation[] observedPoses, Pose2d actualRobotPose) {
    // Get the distance closest to the current robot pose if multiple poses are determined for a
    // camera
    double minDistance = Double.POSITIVE_INFINITY;
    PoseObservation selectedPose = null;
    Pose3d actualRobotPose3d = new Pose3d(actualRobotPose);

    for (var pose : observedPoses) {
      double distance =
          pose.getObservedPose().getTranslation().getDistance(actualRobotPose3d.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        selectedPose = pose;
      }
    }
    return selectedPose;
  }

  private boolean checkPose(PoseObservation observation, int cameraIndex) {
    boolean rejectPose =
        observation.tagCount() == 0 // Must have at least one tag
            || (observation.tagCount() == 1
                && observation.ambiguity()
                    > io[cameraIndex]
                        .getVisionConstants()
                        .maxAmbiguity()) // Cannot be high ambiguity
            || Math.abs(observation.observedPose().getZ())
                > io[cameraIndex]
                    .getVisionConstants()
                    .maxZError() // Must have realistic Z coordinate
            // Must be within the field boundaries
            || observation.observedPose().getX() < 0.0
            || observation.observedPose().getX() > VisionConstants.aprilTagLayout.getFieldLength()
            || observation.observedPose().getY() < 0.0
            || observation.observedPose().getY() > VisionConstants.aprilTagLayout.getFieldWidth()
            || !io[cameraIndex].hasTargets;
    return rejectPose;
  }
}
