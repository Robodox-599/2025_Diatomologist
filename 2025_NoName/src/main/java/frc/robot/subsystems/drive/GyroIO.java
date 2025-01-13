package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class GyroIO {
  /** Updates the set of loggable inputs. */
  public void updateInputs() {}

  protected boolean connected = false;
  protected Rotation2d yawPosition = new Rotation2d();
  protected double yawVelocityRadPerSec = 0.0;
  protected double[] odometryYawTimestamps = new double[] {};
  protected Rotation2d[] odometryYawPositions = new Rotation2d[] {};
}
