package frc.robot.subsystems.vision4.camera;

import frc.robot.subsystems.vision4.camera.Camera.CameraConstants;
import org.photonvision.targeting.PhotonPipelineResult;

public abstract class CameraIO {
  public PhotonPipelineResult result = new PhotonPipelineResult();
  public boolean stale = true;

  public void updateInputs(CameraIOReal inputs) {}

  public String getName() {
    return "";
  }

  public CameraConstants getCameraConstants() {
    return null;
  }
}
