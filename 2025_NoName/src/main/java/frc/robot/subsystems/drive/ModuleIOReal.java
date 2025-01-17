// Copyright 2021-2024 FRC 6328
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

import static frc.robot.subsystems.drive.constants.RealConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.Module.ModuleConstants;
import frc.robot.subsystems.drive.constants.RealConstants;
import frc.robot.util.MotorLog;

import java.util.Queue;

// Class for interfacing with Talon FX motor controllers and CANcoders
// Each instance corresponds to one swerve module

public class ModuleIOReal extends ModuleIO {

  // Drive and turn motor controllers also absolute encoder
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Timestampt to queue to track the time at which measurments are taken
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;
  private final String name;

  private final VoltageOut driveVoltage = new VoltageOut(0.0);
  private final VoltageOut turnVoltage = new VoltageOut(0.0);
  private final VelocityVoltage driveCurrentVelocity = new VelocityVoltage(0.0).withSlot(0);
  private final PositionVoltage turnPID = new PositionVoltage(0.0).withSlot(0);
private final ModuleConstants constants;
  private final double kAVoltsPerMeterPerSecondSquared;

  // Offset angle for the CANcoder to calibrate to zero
  private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();

  public ModuleIOReal(ModuleConstants constants) {
    // Configuration of drive and turn Talon FX motors and CANcoder on module index
    name = constants.prefix();
    this.constants = constants;
    driveTalon = new TalonFX(constants.driveID(), constants.CANBusName());
    turnTalon = new TalonFX(constants.steerID(), constants.CANBusName());
    cancoder = new CANcoder(constants.cancoderID(), constants.CANBusName());

    // Current limits
    driveConfig.CurrentLimits.SupplyCurrentLimit = 35.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Inverts
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Sensor
    // Meters per second
    driveConfig.Feedback.SensorToMechanismRatio = RealConstants.DRIVE_ROTOR_TO_METERS;
    // Voltage Controls Gains
    driveConfig.Slot0.kV = 2.381;
    kAVoltsPerMeterPerSecondSquared = 0.65;
    driveConfig.Slot0.kA = kAVoltsPerMeterPerSecondSquared;
    driveConfig.Slot0.kS = 0.04;
    driveConfig.Slot0.kP = 2.0;
    driveConfig.Slot0.kD = 0.2;

    // Current control gains
    driveConfig.Slot1.kV = 0.0;
    driveConfig.Slot1.kA = 3.07135116146;
    driveConfig.Slot1.kS = 14.0;
    driveConfig.Slot1.kP = 100.0;
    driveConfig.Slot1.kD = 1.0;

    driveConfig.TorqueCurrent.TorqueNeutralDeadband = 10.0;

    driveConfig.MotionMagic.MotionMagicCruiseVelocity = MAX_LINEAR_SPEED;
    driveConfig.MotionMagic.MotionMagicAcceleration = MAX_LINEAR_ACCELERATION;
    driveConfig.MotionMagic.MotionMagicJerk = MAX_LINEAR_ACCELERATION / 0.1;
    driveTalon.getConfigurator().apply(driveConfig);

    // Current limits
    turnConfig.CurrentLimits.StatorCurrentLimit = TURN_STATOR_CURRENT_LIMIT;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Inverts
    turnConfig.MotorOutput.Inverted =
        RealConstants.IS_TURN_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turnConfig.Feedback.RotorToSensorRatio = RealConstants.TURN_GEAR_RATIO;
    turnConfig.Feedback.SensorToMechanismRatio = 1.0;
    turnConfig.Feedback.FeedbackRotorOffset =
        0.0; // Is this correct? Cancoder config should handle it

    // Controls Gains
    turnConfig.Slot0.kV = 0.4;
    turnConfig.Slot0.kS = 0.14;
    turnConfig.Slot0.kP = 105;
    turnConfig.Slot0.kD = 2.15;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 6000 / TURN_GEAR_RATIO;
    turnConfig.MotionMagic.MotionMagicAcceleration = (6000 * 0.1) / TURN_GEAR_RATIO;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.cancoderID();

    turnTalon.getConfigurator().apply(turnConfig);

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = constants.cancoderOffset().getRotations();
    cancoderConfig.MagnetSensor.SensorDirection =
        IS_TURN_MOTOR_INVERTED
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    // Initialize timestamp and position queues from odometry
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(drivePosition);
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnPosition);
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    // Set update frequencies for status signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    // optimize comms between Talons and CAN bus
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
  }

  // Method to update Inputs
  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    super.drivePositionMeters = drivePosition.getValueAsDouble();
    super.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble();
    super.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    super.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};
    super.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());

    super.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    super.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    super.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    super.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    super.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

    super.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    super.odometryDrivePositionsMeters =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    super.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    MotorLog.log("Drive/Module " + constants.prefix() + "/DriveMotor", driveTalon);
    DogLog.log(
        "Drive/Module " + constants.prefix() + "/DriveMotor/Connected", super.driveConnected);

    MotorLog.log("Drive/Module " + constants.prefix() + "/TurnMotor", turnTalon);
    DogLog.log("Drive/Module " + constants.prefix() + "/TurnMotor/Connected", super.turnConnected);

    DogLog.log("Drive/Module " + constants.prefix() + "/Encoder/Connected", super.encoderConnected);
    DogLog.log(
        "Drive/Module " + constants.prefix() + "/Encoder/AbsolutePosition", super.turnAbsolutePosition);

    DogLog.log(
        "Drive/Module " + constants.prefix() + "/Odometry/Timestamps", super.odometryTimestamps);
    DogLog.log(
        "Drive/Module " + constants.prefix() + "/Odometry/DrivePositionsMeters",
        super.odometryDrivePositionsMeters);
    DogLog.log(
        "Drive/Module " + constants.prefix() + "/Odometry/TurnPositions",
        super.odometryTurnPositions);

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(final double volts, final boolean focEnabled) {
    driveTalon.setControl(driveVoltage.withOutput(volts).withEnableFOC(focEnabled));
  }

  @Override
  public void setTurnVoltage(final double volts) {
    turnTalon.setControl(turnVoltage.withOutput(volts));
  }

  @Override
  public void setDriveSetpoint(final double metersPerSecond, final double metersPerSecondSquared) {
    // Doesnt actually refresh drive velocity signal, but should be cached
    if (metersPerSecond == 0
        && metersPerSecondSquared == 0
        && MathUtil.isNear(0.0, driveVelocity.getValueAsDouble(), 0.1)) {
      setDriveVoltage(0.0);
    } else {
      driveTalon.setControl(
          driveCurrentVelocity
              .withVelocity(metersPerSecond)
              .withFeedForward(metersPerSecondSquared * kAVoltsPerMeterPerSecondSquared));
    }
  }

  @Override
  public void setTurnSetpoint(final Rotation2d rotation) {
    turnTalon.setControl(turnPID.withPosition(rotation.getRotations()));
  }

  public String getModuleName() {
    return name;
  }

  @Override
  public void setBrake() {
    turnTalon.setNeutralMode(NeutralModeValue.Brake);
    driveTalon.setNeutralMode(NeutralModeValue.Brake);
  }
}
