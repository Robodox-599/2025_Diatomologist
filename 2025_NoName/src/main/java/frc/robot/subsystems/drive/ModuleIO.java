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
import frc.robot.subsystems.drive.Module.ModuleConstants;

public abstract class ModuleIO {
  protected boolean driveConnected = false;
  protected boolean turnConnected = false;
  protected boolean encoderConnected = false;

  protected double drivePositionMeters = 0.0;
  protected double driveVelocityMetersPerSec = 0.0;
  protected double driveAppliedVolts = 0.0;
  protected double driveCurrentAmps = 0.0;

  protected Rotation2d turnAbsolutePosition = new Rotation2d();
  protected Rotation2d turnPosition = new Rotation2d();
  protected double turnVelocityRadPerSec = 0.0;
  protected double turnAppliedVolts = 0.0;
  protected double turnCurrentAmps = 0.0;

  protected double[] odometryTimestamps = new double[] {};
  protected double[] odometryDrivePositionsMeters = new double[] {};
  protected Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  protected ModuleConstants constants;

  /** Updates the set of loggable inputs. */
  public void updateInputs() {}
  ;

  /** Run the drive motor at the specified voltage. */
  public void setDriveVoltage(final double volts) {
    setDriveVoltage(volts, true);
  }

  /** Run the drive motor at the specified voltage. */
  public void setDriveVoltage(final double volts, final boolean focEnabled) {}
  ;

  /** Use onboard PID to run the drive motor at the specified speed */
  public void setDriveSetpoint(final double metersPerSecond) {}
  ;

  /** Run the turn motor at the specified voltage. */
  public void setTurnVoltage(final double volts) {}
  ;

  /** Use onboard PID to run the turn motor to the specified rotation */
  public void setTurnSetpoint(final Rotation2d rotation) {}
  ;

  /** Gets the name of the swerve module for logging purposes, should be constant per-module. */
  public String getModuleName() {
    return "";
  }
  ;

  public void setBrake() {}
}
