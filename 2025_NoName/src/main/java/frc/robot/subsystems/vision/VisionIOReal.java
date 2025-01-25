package frc.robot.subsystems.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOReal extends VisionIO {
  protected final PhotonCamera camera;
  protected final VisionConstants constants;
  protected final Transform3d robotToCamera;

  /**
   * Buffer which keeps track of the robot rotation over the past few seconds This allows us to
   * match a vision estimate (which are determined with some delay) with a robot rotation
   */
  TimeInterpolatableBuffer<Rotation2d> rotationBuffer = TimeInterpolatableBuffer.createBuffer(1.5);

  /**
   * Creates a new VisionIOReal
   *
   * @param The VisionConstants of the camera.
   */
  public VisionIOReal(VisionConstants cameraConstants) {
    this.constants = cameraConstants;
    camera = new PhotonCamera(cameraConstants.cameraName());
    this.robotToCamera = cameraConstants.robotToCameraTransform3d();
  }

  @Override
  public void updateInputs() {
    super.cameraConnected = camera.isConnected();

    List<PoseObservation> poseObservations = new LinkedList<>();
    List<PhotonPipelineResult> resultList = camera.getAllUnreadResults();
    if (camera == null
        || !camera.isConnected()
        || resultList == null
        || resultList.isEmpty()
        || !resultList.get(resultList.size() - 1).hasTargets()) {
      super.hasTargets = false;
      return;
    }
    super.hasTargets = true;

    // Retrieve the most recent result from the total list
    PhotonPipelineResult latestResult = resultList.get(resultList.size() - 1);
    List<PhotonTrackedTarget> visibleTags = latestResult.getTargets();

    // Fill in the tag area and visible ID arrays
    int[] ID = new int[visibleTags.size()];
    for (int i = 0; i < visibleTags.size(); i++) {
      ID[i] = visibleTags.get(i).getFiducialId();
    }
    super.tagIds = ID;
    for (var result : resultList) {
      // Update latest target observation
      if (result.hasTargets()) {
        super.latestTargetAngle =
            new ObservedTargetRotations(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        super.latestTargetAngle = new ObservedTargetRotations(new Rotation2d(), new Rotation2d());
      }
      // Determine the possible robot pose results the camera has determined
      if (visibleTags.size() == 1) {
        retrieveSingleTagEstimates(latestResult);
      } else {
        retrieveMultiTagEstimates(latestResult);
      }
    }
    for (int i = 0; i < poseObservations.size(); i++) {
      super.poseObservations[i] = poseObservations.get(i);
    }
  }

  // // Read new camera observations
  // Set<Short> tagIds = new HashSet<>();
  // List<PoseObservation> poseObservations = new LinkedList<>();
  // for (var result : camera.getAllUnreadResults()) {
  //   // Update latest target observation
  //   if (result.hasTargets()) {
  //     super.latestTargetAngle =
  //         new ObservedTargetRotations(
  //             Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
  //             Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
  //   } else {
  //     super.latestTargetAngle = new ObservedTargetRotations(new Rotation2d(), new Rotation2d());
  //   }

  //   // Add pose observation
  //   if (result.multitagResult.isPresent()) {
  //     var multitagResult = result.multitagResult.get();

  //     // Calculate robot pose
  //     Transform3d fieldToCamera = multitagResult.estimatedPose.best;
  //     Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
  //     Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

  //     // Calculate average tag distance
  //     double totalTagDistance = 0.0;
  //     for (var target : result.targets) {
  //       totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
  //     }

  //     // Add tag IDs
  //     tagIds.addAll(multitagResult.fiducialIDsUsed);

  //     // Add observation
  // poseObservations.add(
  //     new PoseObservation(
  //         result.getTimestampSeconds(), // Timestamp
  //         robotPose, // 3D pose estimate
  //         multitagResult.estimatedPose.ambiguity, // Ambiguity
  //         multitagResult.fiducialIDsUsed.size(), // Tag count
  //         totalTagDistance / result.targets.size())); // Average tag distance

  //     super.numTargets = result.targets.size();
  //   }
  // }

  // // Save pose observations to inputs
  // super.poseObservations = new PoseObservation[poseObservations.size()];
  // for (int i = 0; i < poseObservations.size(); i++) {
  //   super.poseObservations[i] = poseObservations.get(i);
  // }

  // // Save tag IDs to inputs
  // super.aprilTagIds = new int[tagIds.size()];
  // int i = 0;
  // for (int id : tagIds) {
  //   super.aprilTagIds[i++] = id;
  // }
  // }
  //

  private List<PoseObservation> retrieveSingleTagEstimates(PhotonPipelineResult latestResult) {
    List<PoseObservation> poseObservations = new ArrayList<>();
    PhotonTrackedTarget target = latestResult.getBestTarget();

    Optional<Pose3d> tagPoseOptional =
        VisionConstants.aprilTagLayout.getTagPose(target.getFiducialId());
    Optional<Rotation2d> robotRotationOptional =
        rotationBuffer.getSample(latestResult.getTimestampSeconds());

    if (tagPoseOptional.isPresent() && robotRotationOptional.isPresent()) {
      Pose3d tagPose = tagPoseOptional.get();
      Rotation3d tagRotation = tagPose.getRotation();
      Rotation2d robotRotation = robotRotationOptional.get();

      Rotation3d robotToTargetRot =
          tagRotation.minus(new Rotation3d(0, 0, robotRotation.getRadians()));
      Rotation3d cameraToTargetRot =
          robotToTargetRot.plus(constants.robotToCameraTransform3d().getRotation());

      Transform3d[] camToTargetOptions = {
        new Transform3d(target.getBestCameraToTarget().getTranslation(), cameraToTargetRot),
        new Transform3d(target.getAlternateCameraToTarget().getTranslation(), cameraToTargetRot)
      };

      for (Transform3d camToTarget : camToTargetOptions) {
        Pose3d robotPose =
            PhotonUtils.estimateFieldToRobotAprilTag(
                camToTarget, tagPose, constants.robotToCameraTransform3d());

        poseObservations.add(
            new PoseObservation(
                latestResult.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                target.getPoseAmbiguity(), // Ambiguity
                1, // Tag count (single tag estimate)
                getAverageTagDistance(latestResult) // Average tag distance
                ));
      }

      logDeltaRotation(tagPose.plus(target.getBestCameraToTarget().inverse()));
    }

    return poseObservations;
  }

  private List<PoseObservation> retrieveMultiTagEstimates(PhotonPipelineResult latestResult) {
    List<PoseObservation> poseObservations = new ArrayList<>();
    MultiTargetPNPResult multitagResult = latestResult.getMultiTagResult().get();

    Transform3d[] multiPoseOptions = {
      multitagResult.estimatedPose.best, multitagResult.estimatedPose.alt
    };

    for (Transform3d multiPose : multiPoseOptions) {
      Pose3d robotPose =
          new Pose3d().transformBy(multiPose).transformBy(constants.robotToCameraTransform3d());

      poseObservations.add(
          new PoseObservation(
              latestResult.getTimestampSeconds(), // Timestamp
              robotPose, // 3D pose estimate
              multitagResult.estimatedPose.ambiguity, // Ambiguity
              multitagResult.fiducialIDsUsed.size(), // Tag count
              getAverageTagDistance(
                  latestResult) // You might want to calculate the average tag distance if needed
              ));
    }

    logDeltaRotation(new Pose3d().transformBy(multiPoseOptions[0]));

    return poseObservations;
  }

  /**
   * Log the difference between the camera rotation entered in constants and the camera rotation
   * measured by the vision system. A positive error means that the camera rotation needs to be
   * reduced in constants.
   *
   * @param fieldSpaceCameraPose The detected pose of the camera in field space
   */
  private void logDeltaRotation(Pose3d fieldSpaceCameraPose) {
    Rotation3d rotation = fieldSpaceCameraPose.getRotation();
    Rotation3d cameraRotation = constants.robotToCameraTransform3d().getRotation().unaryMinus();
    DogLog.log(
        "Vision/" + constants.cameraName() + "/Camera Rotation Error/x (roll, deg)",
        cameraRotation.getMeasureX().minus(rotation.getMeasureX()).in(Units.Degree));
    DogLog.log(
        "Vision/" + constants.cameraName() + "/Camera Rotation Error/y (pitch, deg)",
        cameraRotation.getMeasureY().minus(rotation.getMeasureY()).in(Units.Degree));
  }

  private double getAverageTagDistance(PhotonPipelineResult result) {
    double totalTagDistance = 0.0;
    for (var target : result.targets) {
      totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
    }
    return totalTagDistance / result.targets.size();
  }

  // Returns name of the camera
  @Override
  public String getName() {
    return camera.getName();
  }

  // Returns given VisionConstants from Instanization
  @Override
  public VisionConstants getVisionConstants() {
    return constants;
  }
}
