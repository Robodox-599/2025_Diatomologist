// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class ModuleIO {
  protected boolean driveConnected = false;
  protected double drivePositionRad = 0.0;
  protected double driveVelocityRadPerSec = 0.0;
  protected double driveAppliedVolts = 0.0;
  protected double driveCurrentAmps = 0.0;
  protected boolean turnConnected = false;
  protected boolean encoderConnected = false;
  protected Rotation2d absolutePosition = new Rotation2d();
  protected Rotation2d turnPosition = new Rotation2d();
  protected double turnVelocityRadPerSec = 0.0;
  protected double turnAppliedVolts = 0.0;
  protected double turnCurrentAmps = 0.0;
  protected double[] odometryTimestamps = new double[] {};
  protected double[] odometryDrivePositionsRad = new double[] {};
  protected Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

  /** Updates the set of loggable inputs. */
  public void updateInputs() {}

  /** Run the drive motor at the specified open loop value. */
  public void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public void setTurnPosition(Rotation2d rotation) {}
}
