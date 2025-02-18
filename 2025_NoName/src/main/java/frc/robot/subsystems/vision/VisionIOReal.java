package frc.robot.subsystems.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AprilTags;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
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
    Translation3d tagLoc =
        FieldConstants.AprilTags.TAGS[target.fiducialId - 1].pose.getTranslation();
    Transform3d cameraToTag = target.getBestCameraToTarget();
    Translation3d tagToRobotOffset =
        robotToCameraPoseOffset.transformBy(cameraToTag).getTranslation();
    tagToRobotOffset = tagToRobotOffset.rotateBy(new Rotation3d(gyroAngle));

    // Update the latestTargetAngle with the tag's x and y rotations
    Rotation3d tagRotation =
        FieldConstants.AprilTags.TAGS[target.fiducialId - 1].pose.getRotation();
    super.latestTargetAngle =
        new ObservedTargetRotations(
            new Rotation2d(tagRotation.getX()), new Rotation2d(tagRotation.getY()));

    return new Pose3d(tagLoc.minus(tagToRobotOffset), new Rotation3d(gyroAngle));
  }

  public Optional<PoseObservation> updateTest(
      EstimatedRobotPose estRoboPose, List<PhotonPipelineResult> resultList) {
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
    Pose3d pose = estRoboPose.estimatedPose;
    // if (estRoboPose.targetsUsed.size() == 1) {
    //   var target = estRoboPose.targetsUsed.get(0);
    //   avgDistance = target.getBestCameraToTarget().getTranslation().getNorm();
    //   pose = reproject(target, poseSupplier.get().getRotation());
    // } else {
    avgDistance =
        estRoboPose.targetsUsed.stream()
            .map(PhotonTrackedTarget::getBestCameraToTarget)
            .map(Transform3d::getTranslation)
            .map(t3 -> Math.hypot(t3.getX(), t3.getY()))
            .mapToDouble(Double::doubleValue)
            .average()
            .orElseGet(() -> 100.0);
    // }

    // PhotonTrackedTarget latestResult = resultList.get(resultList.size() - 1).getBestTarget();
    // PoseObservation latestUpdate;
    // if (latestResult != null) {
    //   latestUpdate == 3;
    //       new PoseObservation(
    //           estRoboPose.timestampSeconds,
    //           pose,
    //           latestResult.getPoseAmbiguity(),
    //           getSeenTags(),
    //           avgDistance);
    // } else {
    //   latestUpdate =
    //       new PoseObservation(estRoboPose.timestampSeconds, pose, 0.5, getSeenTags(),
    // avgDistance);
    // }

    // previousUpdate = Optional.of(latestUpdate);

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

    if (camera == null || !camera.isConnected() || resultList == null || resultList.isEmpty()) {
      super.hasTargets = false;
      return;
    }

    super.hasTargets = true;
    super.numTargets = seenTags.size();
    super.tagIds = seenTags.stream().mapToInt(i -> i).toArray();
    seenTags.clear();
    super.previousUpdate = previousUpdate;
    resultList.stream()
        .filter(result -> result.hasTargets())
        .map(result -> poseEstimator.update(result))
        .filter(Optional::isPresent)
        .map(Optional::get)
        .map(estRoboPose -> updateTest(estRoboPose, resultList))
        .filter(Optional::isPresent)
        .map(Optional::get)
        .forEach(poseObservations::add);

    // Determine the possible robot pose results the camera has determined
    super.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      super.poseObservations[i] = poseObservations.get(i);
    }
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
