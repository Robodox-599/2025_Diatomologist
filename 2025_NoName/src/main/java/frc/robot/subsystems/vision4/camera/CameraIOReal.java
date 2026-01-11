package frc.robot.subsystems.vision4.camera;

import frc.robot.subsystems.vision4.camera.Camera.CameraConstants;
import org.photonvision.PhotonCamera;

public class CameraIOReal extends CameraIO {

  private final PhotonCamera camera;
  private final CameraConstants constants;

  public CameraIOReal(CameraConstants constants) {
    this.constants = constants;
    this.camera = new PhotonCamera(constants.name());
  }

  @Override
  public void updateInputs() {
    super.result = camera.getLatestResult();
    result.hasTargets();
  }

  @Override
  public CameraConstants getCameraConstants() {
    return constants;
  }

  @Override
  public String getName() {
    return constants.name();
  }
}
