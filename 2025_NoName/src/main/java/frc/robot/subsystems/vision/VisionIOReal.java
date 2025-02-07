package frc.robot.subsystems.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AprilTags;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOReal extends VisionIO {
  protected final PhotonCamera camera;
  protected final VisionConstants constants;
  protected final Transform3d robotToCamera;
  private final Pose3d robotToCameraPoseOffset;
  private final Supplier<Pose2d> poseSupplier;
  private final PhotonPoseEstimator poseEstimator;

  private Optional<PoseObservation> previousUpdate = Optional.empty();
  private ArrayList<Integer> seenTags = new ArrayList<>();
  private ArrayList<PoseObservation> poseObservations = new ArrayList<>();

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
  public VisionIOReal(VisionConstants cameraConstants, Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    this.constants = cameraConstants;
    super.constants = constants;

    camera = new PhotonCamera(cameraConstants.cameraName());

    this.robotToCamera = cameraConstants.robotToCameraTransform3d();
    this.robotToCameraPoseOffset = Pose3d.kZero.transformBy(robotToCamera);

    poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.AprilTags.aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            this.robotToCamera);
    poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimator.setPrimaryStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    DogLog.log("Vision/" + camera.getName() + "/Camera Transform", robotToCamera);
  }
  private Pose3d reproject(PhotonTrackedTarget target, Rotation2d gyroAngle) {
    Translation2d tagLoc = FieldConstants.AprilTags.TAGS_POSE2D[target.fiducialId - 1].getTranslation();
    Transform3d cameraToTag = target.getBestCameraToTarget();
    Translation2d tagToRobotOffset =
        robotToCameraPoseOffset.transformBy(cameraToTag).toPose2d().getTranslation();
    tagToRobotOffset = tagToRobotOffset.rotateBy(gyroAngle);
    return new Pose3d(new Translation3d(tagLoc.minus(tagToRobotOffset).getX(), tagLoc.minus(tagToRobotOffset).getY(), ), gyroAngle);
  }


  public Optional<PoseObservation> updateTest(EstimatedRobotPose estRoboPose, List<PhotonPipelineResult> resultList) {
    for (PhotonTrackedTarget target : estRoboPose.targetsUsed) {
      int minId = AprilTags.TAGS[0].ID;
      int maxId = AprilTags.TAGS[AprilTags.TAGS.length - 1].ID;
      if (target.fiducialId < minId && target.fiducialId > maxId) {
        previousUpdate = Optional.empty();
        return previousUpdate;
      }
      seenTags.add(target.fiducialId);
    }

    double avgDistance;
    Pose2d pose = estRoboPose.estimatedPose.toPose2d();
    if (estRoboPose.targetsUsed.size() == 1) {
      var target = estRoboPose.targetsUsed.get(0);
      avgDistance = target.getBestCameraToTarget().getTranslation().getNorm();
      pose = reproject(target, poseSupplier.get().getRotation());
    } else {
      avgDistance =
          estRoboPose.targetsUsed.stream()
              .map(PhotonTrackedTarget::getBestCameraToTarget)
              .map(Transform3d::getTranslation)
              .map(t3 -> Math.hypot(t3.getX(), t3.getY()))
              .mapToDouble(Double::doubleValue)
              .average()
              .orElseGet(() -> 100.0);
    }


    PhotonTrackedTarget latestResult = resultList.get(resultList.size() - 1).getBestTarget();
    
    PoseObservation latestUpdate =
        new PoseObservation(
            estRoboPose.timestampSeconds,
            previousUpdate.get().getObservedPose(),
            latestResult.getPoseAmbiguity(),
            getSeenTags(),
            avgDistance);

    previousUpdate = Optional.of(latestUpdate);

    return previousUpdate;
  }

  public List<Integer> getSeenTags() {
    return seenTags;
  }

  @Override
  public void updateInputs() {
    super.cameraConnected = camera.isConnected();
    List<PoseObservation> poseObservations = new LinkedList<>();
    List<PhotonPipelineResult> resultList = camera.getAllUnreadResults();
    Set<Short> tagIds = new HashSet<>();
    
    if (camera == null || !camera.isConnected() || resultList == null || resultList.isEmpty()) {
        super.hasTargets = false;
        return;
    }
    
    super.hasTargets = true;
    
    // Retrieve the most recent result from the total list
    PhotonPipelineResult latestResult = resultList.get(resultList.size() - 1);
    List<PhotonTrackedTarget> visibleTags = latestResult.getTargets();
    
    seenTags.clear();
    resultList.stream()
        .filter(result -> result.hasTargets())
        .map(result -> poseEstimator.update(result))
        .filter(Optional::isPresent)
        .map(Optional::get)
        .map(estRoboPose -> updateTest(estRoboPose, resultList))
        .filter(Optional::isPresent)
        .map(Optional::get)
        .forEach(poseObservations::add);
    
    super.tagIds = new int[visibleTags.size()];
    for (int i = 0; i < visibleTags.size(); i++) {
        super.tagIds[i] = visibleTags.get(i).getFiducialId();
    }
    
    // Determine the possible robot pose results the camera has determined
    super.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
        super.poseObservations[i] = poseObservations.get(i);
    }
  }


  /**
   * Log the difference between the camera rotation entered in constants and the camera rotation
   * measured by the vision system. A positive error means that the camera rotation needs to be
   * reduced in constants.
   *
   * @param cameraPose The detected pose of the camera in field space
   */
  private void logDeltaRotation(Pose3d cameraPose) {
    Rotation3d rotation = cameraPose.getRotation();
    Rotation3d cameraRotation = constants.robotToCameraTransform3d().getRotation().unaryMinus();
    DogLog.log(
        "Vision/" + constants.cameraName() + "/Camera Rotation Error/x (roll, deg)",
        cameraRotation.getMeasureX().minus(rotation.getMeasureX()).in(Units.Degree));
    DogLog.log(
        "Vision/" + constants.cameraName() + "/Camera Rotation Error/y (pitch, deg)",
        cameraRotation.getMeasureY().minus(rotation.getMeasureY()).in(Units.Degree));
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
