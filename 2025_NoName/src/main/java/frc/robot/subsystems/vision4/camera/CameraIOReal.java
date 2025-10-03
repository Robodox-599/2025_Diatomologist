package frc.robot.subsystems.vision4.camera;

import frc.robot.subsystems.vision4.camera.Camera.CameraConstants;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraIOReal extends CameraIO {

  private final PhotonCamera camera;
  private final CameraConstants constants;

  public CameraIOReal(CameraConstants constants) {
    this.constants = constants;
    this.camera = new PhotonCamera(constants.name());
  }

  @Override
  public void updateInputs(CameraIOReal inputs) {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (results.size() > 0) {
      super.result = results.get(results.size() - 1);
    }
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
