package frc.robot.subsystems.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final SwerveConsumer speedsConsumer;

  private final VisionIO[] io;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, SwerveConsumer speedsConsumer, VisionIO... io) {
    this.consumer = consumer;
    this.speedsConsumer = speedsConsumer;
    this.io = io;

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      disconnectedAlerts[i] = new Alert(io[i].getName() + " is disconnected.", AlertType.kWarning);
    }
  }

  public void updateInputs() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs();
    }

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {

      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!io[cameraIndex].cameraConnected);

      // Loop over pose observations
      for (var observation : io[cameraIndex].poseObservations) {
        // Calculate standard deviations for selected pose
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.getTagCount();

        double linearStdDev =
            io[cameraIndex].getVisionConstants().linearStdDevBaseline() * stdDevFactor;
        double angularStdDev =
            io[cameraIndex].getVisionConstants().angularStdDevBaseline() * stdDevFactor;

        linearStdDev *= io[cameraIndex].getVisionConstants().cameraStdDevFactor();
        angularStdDev *= io[cameraIndex].getVisionConstants().cameraStdDevFactor();
        
        // Check whether to reject pose
        boolean rejectPose = checkPose(observation, cameraIndex);

        var speeds = speedsConsumer.getSpeeds();

        if (observation.getTagArea() > 8
        && speeds.vxMetersPerSecond < 3
        && speeds.vyMetersPerSecond < 3
        && speeds.omegaRadiansPerSecond < 4 * Math.PI && !rejectPose) {
          DogLog.log("Vision/" + io[cameraIndex].getName() + "/PoseAccepted?", true);
          DogLog.log(
          "Vision/" + io[cameraIndex].getName() + "/AcceptedPoseObservation",
          observation.getObservedPose());

          angularStdDev += 25;
        } else {
          DogLog.log("Vision/" + io[cameraIndex].getName() + "/PoseAccepted?", false);
          DogLog.log(
              "Vision/" + io[cameraIndex].getName() + "/RejectedRobotPose",
              observation.observedPose());

          continue;
        }

        consumer.accept(
            observation.getObservedPose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
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

  @FunctionalInterface
  public static interface SwerveConsumer {
    public ChassisSpeeds getSpeeds();
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
            > io[cameraIndex].getVisionConstants().getMaxZError()
        || pose.getRotation().getY() > io[cameraIndex].getVisionConstants().getMaxAngleError()
        || pose.getRotation().getX() > io[cameraIndex].getVisionConstants().getMaxAngleError()
        || observation.getAverageTagDistance() > 5.5);
  }

  public void logValues(int cameraIndex) {
    DogLog.log(
        "Vision/" + io[cameraIndex].getName() + "/CameraConnected",
        io[cameraIndex].cameraConnected);
    DogLog.log("Vision/" + io[cameraIndex].getName() + "/HasTargets", io[cameraIndex].hasTargets);
    DogLog.log("Vision/" + io[cameraIndex].getName() + "/NumTargets", io[cameraIndex].numTargets);
    DogLog.log("Vision/" + io[cameraIndex].getName() + "/TagIds", io[cameraIndex].tagIds);
  }
}
