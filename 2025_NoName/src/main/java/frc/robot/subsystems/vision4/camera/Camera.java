// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision4.camera;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.util.SubsystemChecker;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera {
  public record CameraConstants(String name, Transform3d robotToCamera) {}

  private final PhotonPoseEstimator poseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.AprilTags.aprilTagFieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          null);
  private final CameraIO io;
  private final VisionConsumer consumer;
  private final SubsystemChecker subsystemChecker;

  public static final Matrix<N3, N1> visionPointBlankDevs =
      new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] {0.6, 0.6, 0.5}); // 0.6, 0.6, 0.5
  public static final Matrix<N3, N1> invalidDevs =
      new Matrix<N3, N1>(
          Nat.N3(), Nat.N1(), new double[] {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE});
  public static final double distanceFactor = 3.0;

  public Camera(CameraIOReal io, VisionConsumer consumer, SubsystemChecker subsystemChecker) {
    this.io = io;
    poseEstimator.setRobotToCameraTransform(io.getCameraConstants().robotToCamera());
    this.consumer = consumer;
    poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    this.subsystemChecker = subsystemChecker;
  }

  // Functional interface for vision consumer
  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public Optional<EstimatedRobotPose> update(PhotonPipelineResult result) {
    if (result.getTargets().size() < 1) {
      DogLog.log("Vision/Camera/" + io.getName() + "/Targets", 0);
      return Optional.empty();
    }
    DogLog.log("Vision/Camera/" + io.getName() + "/Targets", result.getTargets().size());
    return poseEstimator.update(result);
  }

  public Matrix<N3, N1> findVisionMeasurementStdDevs(EstimatedRobotPose estimation) {
    double sumDistance = 0;
    for (PhotonTrackedTarget target : estimation.targetsUsed) {
      Transform3d t3d = target.getBestCameraToTarget();
      sumDistance += Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2));
    }
    double avgDistance = sumDistance / estimation.targetsUsed.size();

    Matrix<N3, N1> deviation =
        visionPointBlankDevs.times(Math.max(avgDistance, 0.0) * distanceFactor);
    // if (estimation.targetsUsed.size() == 1) {
    //   deviation = deviation.times(3);
    // }
    if (estimation.targetsUsed.size() == 1
        && estimation.targetsUsed.get(0).poseAmbiguity > CameraErrorConstants.maxAmbiguity) {
      return invalidDevs;
    }
    if (DriverStation.isDisabled()) {
      return visionPointBlankDevs.times(0.75);
    }
    ChassisSpeeds speeds = subsystemChecker.getChassisSpeeds();
    deviation =
        deviation.times(
            CameraErrorConstants.LINEAR_VELOCITY_STD_DEV_COEFFICIENT.lerp(
                Math.sqrt(
                    Math.pow(speeds.vxMetersPerSecond, 2)
                        + Math.pow(speeds.vyMetersPerSecond, 2))));
    deviation =
        deviation.times(
            CameraErrorConstants.ANGULAR_VELOCITY_STD_DEV_COEFFICIENT.lerp(
                speeds.omegaRadiansPerSecond));
    // }
    // TAG_COUNT_DEVIATION_PARAMS
    //     .get(
    //         MathUtil.clamp(
    //             estimation.targetsUsed.size() - 1, 0, TAG_COUNT_DEVIATION_PARAMS.size() - 1))
    //     .computeDeviation(avgDistance);
    return deviation;
  }

  public void updateInputs() {
    io.updateInputs();
    Optional<EstimatedRobotPose> estPose = update(io.result);
    if (estPose.isPresent()) {
      Pose3d visionPose = estPose.get().estimatedPose;
      if (isPoseValid(visionPose)) {
        Matrix<N3, N1> deviations = findVisionMeasurementStdDevs(estPose.get());
        DogLog.log("Vision/Camera/" + io.getName() + "/Deviations", deviations);
        DogLog.log("Vision/Camera/" + io.getName() + "/EstimatedPose", visionPose.toPose2d());
        consumer.accept(visionPose.toPose2d(), io.result.getTimestampSeconds(), deviations);
      }
    }
  }

  public String getName() {
    return io.getName();
  }

  private boolean isPoseValid(Pose3d pose) {
    Translation2d simplePose = pose.getTranslation().toTranslation2d();
    return !(simplePose.getX() < 0.0
        || simplePose.getX() > FieldConstants.fieldLength
        || simplePose.getY() < 0.0
        || simplePose.getY() > FieldConstants.fieldWidth
        || Double.isNaN(simplePose.getX())
        || Double.isNaN(simplePose.getY())
        || Math.abs(pose.getTranslation().getZ()) > CameraErrorConstants.maxZError
        || pose.getRotation().getY() > CameraErrorConstants.maxAngleError
        || pose.getRotation().getX() > CameraErrorConstants.maxAngleError);
  }
}
