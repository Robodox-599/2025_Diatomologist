package frc.robot.subsystems.vision3;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision3.Vision3.VisionUpdate;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision3IOReal {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final VisionConstants constants;
  private final Alert disconnectedAlert;

  public Vision3IOReal(VisionConstants constants) {
    this.constants = constants;

    this.camera = new PhotonCamera(constants.cameraName());

    this.poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.AprilTags.aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            constants.robotToCameraTransform3d());
    poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    this.disconnectedAlert =
        new Alert(constants.cameraName() + " is disconnected.", AlertType.kWarning);

    DogLog.log(
        "Vision/" + constants.cameraName() + "/Camera Transform",
        constants.robotToCameraTransform3d());
  }

  public List<VisionUpdate> update() {
    List<VisionUpdate> updates = new ArrayList<VisionUpdate>();

    for (PhotonPipelineResult change : camera.getAllUnreadResults()) {

      Optional<EstimatedRobotPose> visionEst = poseEstimator.update(change);

      if (visionEst.isPresent()) {
        EstimatedRobotPose est = visionEst.get();
        if (isPoseValid(est.estimatedPose)) {
          Matrix<N3, N1> stdDevs = computeEstimationStdDevs(visionEst, change.getBestTarget());
          updates.add(
              new VisionUpdate(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs));
        }
      }
    }
    return updates;
  }

  private Matrix<N3, N1> computeEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, PhotonTrackedTarget target) {
    if (estimatedPose.isEmpty() || target == null) {
      return Vision3Constants.kSingleTagStdDevs;
    } else {
      Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());

      if (tagPose.isEmpty()) {
        return Vision3Constants.kSingleTagStdDevs;
      } else {
        Matrix<N3, N1> estStdDevs = Vision3Constants.kSingleTagStdDevs;
        double distanceFromTag =
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());

        if (distanceFromTag > 4) {
          return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          return estStdDevs.times(1 + (distanceFromTag * distanceFromTag / 30));
        }
      }
    }
  }

  private boolean isPoseValid(Pose3d pose) {
    Translation2d simplePose = pose.getTranslation().toTranslation2d();
    return !(simplePose.getX() < 0.0
        || simplePose.getX() > FieldConstants.fieldLength
        || simplePose.getY() < 0.0
        || simplePose.getY() > FieldConstants.fieldWidth
        || Double.isNaN(simplePose.getX())
        || Double.isNaN(simplePose.getY())
        || Math.abs(pose.getTranslation().getZ()) > 0.2
        || pose.getRotation().getY() > 0.08726646259971647
        || pose.getRotation().getX() > 0.08726646259971647);
  }
}
