// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision4.camera;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.util.SubsystemChecker;
import java.util.ArrayList;
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
          FieldConstants.AprilTags.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, null);
  private final CameraIO io;
  private final VisionConsumer consumer;
  private final SubsystemChecker subsystemChecker;

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

  public void updateInputs() {
    io.updateInputs();

    if (io.result.hasTargets()) {
      if (io.result.targets.size() > 1
          || io.result.targets.get(0).getPoseAmbiguity() < CameraErrorConstants.maxAmbiguity) {
        PhotonPipelineResult result = pruneTags(io.result);
        Optional<EstimatedRobotPose> optionalEstPose = poseEstimator.update(result);
        if (optionalEstPose.isPresent()) {
          Pose3d estimatedPose = optionalEstPose.get().estimatedPose;
          if (isPoseValid(estimatedPose)) {
            Matrix<N3, N1> stdDevs = findVisionMeasurementStdDevs(optionalEstPose.get());
            DogLog.log("Vision/Camera/" + io.getName() + "/StdDevs", stdDevs);
            DogLog.log(
                "Vision/Camera/" + io.getName() + "/EstimatedPose", estimatedPose.toPose2d());
            consumer.accept(estimatedPose.toPose2d(), result.getTimestampSeconds(), stdDevs);
          }
        }
      }
    }
  }

  public Matrix<N3, N1> findVisionMeasurementStdDevs(EstimatedRobotPose estimation) {
    Matrix<N3, N1> estStdDevs = CameraErrorConstants.SINGLE_TAG_STD_DEVS;

    int numTags = 0;
    double sumDistance = 0;
    for (PhotonTrackedTarget target : estimation.targetsUsed) {
      numTags++;
      Transform3d t3d = target.getBestCameraToTarget();
      sumDistance += Math.hypot(t3d.getX(), t3d.getY());
    }

    if (numTags == 0) {
      return estStdDevs;
    }

    double avgDistance = sumDistance / numTags;

    if (numTags > 1) {
      estStdDevs = CameraErrorConstants.MULTI_TAG_STD_DEVS;
    } else {
      estStdDevs =
          VecBuilder.fill(
              CameraErrorConstants.SINGLE_TAG_STD_DEVS.get(0, 0),
              CameraErrorConstants.SINGLE_TAG_STD_DEVS.get(1, 0),
              Double.MAX_VALUE);
    }

    if (numTags == 1 && avgDistance > CameraErrorConstants.SINGLE_TAG_MAX_DISTANCE_METERS) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + ((avgDistance * avgDistance) / 30));
    }
    return estStdDevs;
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

  private PhotonPipelineResult pruneTags(PhotonPipelineResult result) {
    ArrayList<PhotonTrackedTarget> newTargets = new ArrayList<>();
    boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

    if (isBlueAlliance) {
      for (var target : result.targets) {
        if (target.fiducialId == 18
            || target.fiducialId == 21
            || target.fiducialId == 24
            || target.fiducialId == 25
            || target.fiducialId == 26
            || target.fiducialId == 27) {
          newTargets.add(target);
        }
      }
    } else {
      for (var target : result.targets) {
        if (target.fiducialId == 2
            || target.fiducialId == 5
            || target.fiducialId == 8
            || target.fiducialId == 9
            || target.fiducialId == 10
            || target.fiducialId == 11) {
          newTargets.add(target);
        }
      }
    }
    result.targets = newTargets;
    return result;
  }
}
