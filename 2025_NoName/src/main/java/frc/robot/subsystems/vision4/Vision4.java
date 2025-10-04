// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision4;

import frc.robot.subsystems.vision4.camera.Camera;

public class Vision4 {
  private final Camera[] cameras;

  public Vision4(Camera... cameras) {
    this.cameras = cameras;
  }

  public void updateInputs() {
    for (Camera camera : cameras) {
      camera.updateInputs();
    }
  }
}
