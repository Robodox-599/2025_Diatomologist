package frc.robot.subsystems.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.constants.RealConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.LinkedList;
import java.util.List;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final Alert[] disconnectedAlerts;

  List<Pose3d> tagPoses = new LinkedList<>();
  List<Pose3d> robotPosesAccepted = new LinkedList<>();
  List<Pose3d> robotPosesRejected = new LinkedList<>();

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
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
      io[i].updateInputs();
    }

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {

      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!io[cameraIndex].cameraConnected);

      // Add tag poses
      for (int tagId : io[cameraIndex].tagIds) {
        var tagPose = FieldConstants.AprilTags.aprilTagFieldLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : io[cameraIndex].poseObservations) {
        // Check whether to reject pose
        PoseObservation[] poseObservationWithPreviousUpdate = new PoseObservation[2];
        poseObservationWithPreviousUpdate[0] = observation;
        poseObservationWithPreviousUpdate[1] = io[cameraIndex].previousUpdate.orElse(null);
        if (poseObservationWithPreviousUpdate[1] == null) {
          continue;
        }
        boolean rejectPose = checkPose(poseObservationWithPreviousUpdate, cameraIndex);
        // Add pose to log
        DogLog.log("Vision/" + io[cameraIndex].getName() + "/PoseAccepted?", !rejectPose);
        if (rejectPose) {
          DogLog.log(
              "Vision/" + io[cameraIndex].getName() + "/RejectedRobotPose",
              observation.observedPose());
          continue;
        } else {
          DogLog.log(
              "Vision/" + io[cameraIndex].getName() + "/AcceptedPoseObservation",
              observation.getObservedPose());
        }

        // Calculate standard deviations for selected pose
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.getTagCount();
        double linearStdDev =
            io[cameraIndex].getVisionConstants().linearStdDevBaseline() * stdDevFactor;
        // double angularStdDev =
        //     io[cameraIndex].getVisionConstants().angularStdDevBaseline() * stdDevFactor;

        linearStdDev *= io[cameraIndex].getVisionConstants().cameraStdDevFactor();
        // angularStdDev *= io[cameraIndex].getVisionConstants().angularStdDevBaseline();

        consumer.accept(
            observation.getObservedPose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, 1000000000));
      }
      logValues(cameraIndex);
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  private boolean checkPose(PoseObservation[] observations, int cameraIndex) {
    PoseObservation latestObservation = observations[0];
    PoseObservation previousPoseObservation = observations[1];
    Pose3d pose = latestObservation.getObservedPose();
    Pose3d previousPose = previousPoseObservation.getObservedPose();
    double time = latestObservation.getTimestamp() - previousPoseObservation.getTimestamp();
    Translation2d simplePose = pose.getTranslation().toTranslation2d();
    boolean outOfBounds =
        simplePose.getX() < 0.0
            || simplePose.getX() > FieldConstants.fieldLength
            || simplePose.getY() < 0.0
            || simplePose.getY() > FieldConstants.fieldWidth
            || Double.isNaN(simplePose.getX())
            || Double.isNaN(simplePose.getY());
    boolean extremeJitter =
        pose.getTranslation().getDistance(previousPose.getTranslation())
            > time * RealConstants.MAX_LINEAR_SPEED;
    boolean infeasibleZValue =
        Math.abs(pose.getTranslation().getZ())
            > io[cameraIndex].getVisionConstants().getMaxZError();
    boolean infeasiblePitchValue =
        pose.getRotation().getY() > io[cameraIndex].getVisionConstants().getMaxAngleError();
    boolean infeasibleRollValue =
        pose.getRotation().getX() > io[cameraIndex].getVisionConstants().getMaxAngleError();
    boolean outOfRange = latestObservation.getAverageTagDistance() > 5.5;
    boolean noTags = latestObservation.tagsList().size() < 0;
    boolean sketchyTags = latestObservation.tagsList().stream().anyMatch(List.of()::contains);

    boolean rejectPose =
        outOfBounds
            || extremeJitter
            || infeasibleZValue
            || infeasiblePitchValue
            || infeasibleRollValue
            || outOfRange
            || noTags
            || sketchyTags;
    return rejectPose;
  }

  public void logValues(int cameraIndex) {
    DogLog.log(
        "Vision/" + io[cameraIndex].getName() + "/CameraConnected",
        io[cameraIndex].cameraConnected);
    DogLog.log("Vision/" + io[cameraIndex].getName() + "/HasTargets", io[cameraIndex].hasTargets);
    DogLog.log("Vision/" + io[cameraIndex].getName() + "/NumTargets", io[cameraIndex].numTargets);
    DogLog.log("Vision/" + io[cameraIndex].getName() + "/TagIds", io[cameraIndex].tagIds);
    DogLog.log(
        "Vision/" + io[cameraIndex].getName() + "/LatestTargetAngleX",
        io[cameraIndex].latestTargetAngle.getTargetX());
    DogLog.log(
        "Vision/" + io[cameraIndex].getName() + "/LatestTargetAngleY",
        io[cameraIndex].latestTargetAngle.getTargetY());
    DogLog.log(
        "Vision/" + io[cameraIndex].getName() + "/TagPoses",
        tagPoses.toArray(new Pose3d[tagPoses.size()]));
  }
}
