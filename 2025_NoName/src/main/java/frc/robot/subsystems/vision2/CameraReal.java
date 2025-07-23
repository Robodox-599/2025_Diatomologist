package frc.robot.subsystems.vision2;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants;
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

public class CameraReal {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final VisionConstants constants;
  private final Alert disconnectedAlert;

  public CameraReal(VisionConstants constants) {
    this.constants = constants;

    // Construct camera
    this.camera = new PhotonCamera(constants.cameraName());

    // Construct pose estimator
    this.poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.AprilTags.aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            constants.robotToCameraTransform3d());
    poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Alert setup
    this.disconnectedAlert =
        new Alert(constants.cameraName() + " is disconnected.", AlertType.kWarning);

    // Logging
    DogLog.log(
        "Vision/" + constants.cameraName() + "/Camera Transform",
        constants.robotToCameraTransform3d());
  }

  public PoseObservation[] update() {
    // Example: update disconnected alert based on connection
    // PoseObservation[] observations = new PoseObservation[0];
    ArrayList<PoseObservation> observations = new ArrayList<PoseObservation>();
    boolean isConnected = camera.isConnected();
    String key = "Vision/" + constants.cameraName();
    DogLog.log(key + "/Connected?", isConnected);
    disconnectedAlert.set(!isConnected);
    if (!isConnected) {
      return new PoseObservation[0]; // Skip processing if camera is disconnected
    }

    // Optionally, estimate global pose
    List<PhotonPipelineResult> resultList = camera.getAllUnreadResults();

    if (resultList.isEmpty()) {
      DogLog.log(key + "/Results", "No unread results available.");
      return new PoseObservation[0]; // No results to process
    }

    DogLog.log(key + "/Results", resultList.size() + " unread results found.");

    int i = 0;

    for (PhotonPipelineResult result : resultList) {
      ArrayList<Integer> targets = new ArrayList<Integer>();
      i++;
      if (result.hasTargets()) {
        key = key + "/Results/Result " + i + " /Targets";
        DogLog.log(key, "Targets in Result");
        Optional<EstimatedRobotPose> optionalRobotPose = poseEstimator.update(result);
        if (optionalRobotPose.isEmpty()) {
          DogLog.log(key + "/Targets Used", false);
          continue;
        }
        DogLog.log(key + "/Targets Used", true);
        EstimatedRobotPose estimatedRobotPose = optionalRobotPose.get();
        Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
        double totalDistance = 0.0;
        double totalArea = 0.0;
        double averageDistance = 0.0;
        double averageTagArea = 0.0;
        double ambiguity = 0.0;
        for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
          targets.add(target.getFiducialId());
          Transform3d transform = target.getBestCameraToTarget();
          Translation3d translation = transform.getTranslation();
          double distance = Math.hypot(translation.getX(), translation.getY());
          totalDistance += distance;
          totalArea += target.getArea();
          ambiguity = target.getPoseAmbiguity();
        }
        averageDistance =
            estimatedRobotPose.targetsUsed.isEmpty() ? 100.0 : totalDistance / targets.size();
        averageTagArea =
            estimatedRobotPose.targetsUsed.isEmpty() ? 0.0 : totalArea / targets.size();

        DogLog.log(key + "/Tags Used", targets.size());
        DogLog.log(key + "/Tags Used/Tag IDs", targets.toString());
        DogLog.log(key + "/Estimated Pose", estimatedPose);
        DogLog.log(key + "/Timestamp", result.getTimestampSeconds());

        // Create a PoseObservation with the estimated pose and targets

        PoseObservation observation =
            new PoseObservation(
                result.getTimestampSeconds(),
                estimatedPose,
                ambiguity,
                targets,
                averageDistance,
                averageTagArea);
        observations.add(observation);
      } else {
        DogLog.log("Vision/Results/" + constants.cameraName() + "Targets", "No Targets in results");
      }
    }
    return observations.toArray(new PoseObservation[0]);
  }

  public VisionConstants getConstants() {
    return constants;
  }
}
