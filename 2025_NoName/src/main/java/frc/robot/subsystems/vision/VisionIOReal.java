package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOReal extends VisionIO {
    protected final PhotonCamera camera;
    protected final VisionConstants constants;
    protected final Transform3d robotToCamera;

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

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        super.latestTargetAngle =
            new ObservedTargetRotations(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        super.latestTargetAngle = new ObservedTargetRotations(new Rotation2d(), new Rotation2d());
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size())); // Average tag distance 

        super.numTargets = result.targets.size();
      }
    }

    // Save pose observations to inputs 
    super.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      super.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs 
    super.aprilTagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      super.aprilTagIds[i++] = id;
    }
  }
  
  // Returns name of the camera
  @Override
  public String getName(){
      return camera.getName();
  }

  // Returns given VisionConstants from Instanization
  @Override
  public VisionConstants getVisionConstants(){
    return constants;
  }


}
