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
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.Module.ModuleConstants;
import frc.robot.subsystems.drive.constants.RealConstants;
import frc.robot.util.MotorLog;
import frc.robot.util.PhoenixUtil;
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
  private final VelocityVoltage drivePID = new VelocityVoltage(0.0).withSlot(0);
  private final PositionVoltage turnPID = new PositionVoltage(0.0).withSlot(0);
  private final ModuleConstants constants;

  // Offset angle for the CANcoder to calibrate to zero
  private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

  public ModuleIOReal(ModuleConstants constants) {

    this.name = constants.prefix();
    this.constants = constants;
    super.constants = constants;
    driveTalon = new TalonFX(constants.driveID(), constants.CANBusName());
    turnTalon = new TalonFX(constants.steerID(), constants.CANBusName());
    cancoder = new CANcoder(constants.cancoderID(), constants.CANBusName());

    /* ************ CURRENT LIMITS ************ */

    driveConfig.CurrentLimits.SupplyCurrentLimit = 35.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.CurrentLimits.StatorCurrentLimit = TURN_STATOR_CURRENT_LIMIT;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    /* ************ DRIVE VOLTAGE-PID CONFIGS ************ */

    driveConfig.Slot0.kV = 0;
    driveConfig.Slot0.kS = 0;
    driveConfig.Slot0.kP = 0.0;
    driveConfig.Slot0.kD = 0.0;

    /* ************ DRIVE TORQUE-CURRENT-FOC-PID CONFIGS ************ */

    driveConfig.Slot1.kV = 0.0;
    driveConfig.Slot1.kA = 0;
    driveConfig.Slot1.kS = 0;
    driveConfig.Slot1.kP = 0;
    driveConfig.Slot1.kD = 0.0;
    driveConfig.TorqueCurrent.TorqueNeutralDeadband = 10.0;

    /* ************ TURN VOLTAGE-PID CONFIGS ************ */

    turnConfig.Slot0.kV = 0.0;
    turnConfig.Slot0.kS = 0.0;
    turnConfig.Slot0.kP = 0;
    turnConfig.Slot0.kD = 0;

    /* ************ MOTION MAGIC CONFIGS ************ */

    // turnConfig.MotionMagic.MotionMagicCruiseVelocity = 6000 / TURN_GEAR_RATIO;
    // turnConfig.MotionMagic.MotionMagicAcceleration = (6000 * 0.1) / TURN_GEAR_RATIO;
    // driveConfig.MotionMagic.MotionMagicCruiseVelocity = MAX_LINEAR_SPEED;
    // driveConfig.MotionMagic.MotionMagicAcceleration = MAX_LINEAR_ACCELERATION;
    // driveConfig.MotionMagic.MotionMagicJerk = MAX_LINEAR_ACCELERATION / 0.1;

    /* ************ INVERTS ************ */

    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turnConfig.MotorOutput.Inverted =
        IS_TURN_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.SensorDirection =
        IS_TURN_MOTOR_INVERTED
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    /* ************ CONVERTS FROM ENCODER POSITION TO METERS PER SECOND. ************ */

    driveConfig.Feedback.SensorToMechanismRatio = RealConstants.DRIVE_ROTOR_TO_METERS;

    /*  ************ APPLY TURN CONFIG SENSOR FEEDBACK INFO ************ */

    turnConfig.Feedback.FeedbackRemoteSensorID = constants.cancoderID();
    turnConfig.Feedback.FeedbackSensorSource =
        FeedbackSensorSourceValue
            .RemoteCANcoder; // change to FeedbackSensorSourceValue.FusedCANCoder;
    turnConfig.Feedback.RotorToSensorRatio = RealConstants.TURN_GEAR_RATIO;
    turnConfig.Feedback.SensorToMechanismRatio = 1.0;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    /* ************ APPLY BRAKE MODES *************/

    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    /* ************ APPLY CANCODER OFFSETS *************/

    cancoderConfig.MagnetSensor.MagnetOffset = constants.cancoderOffset().getRotations();

    /* ************ APPLY CONFIGS ************ */

    PhoenixUtil.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> turnTalon.getConfigurator().apply(driveConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    // Initialize timestamp and position queues from odometry
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePosition);
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnPosition);
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    // Set update frequencies for status signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        RealConstants.ODOMETRY_FREQUENCY, drivePosition, turnPosition);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, cancoder);
  }

  // Method to update Inputs
  @Override
  public void updateInputs() {
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    super.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    super.drivePositionMeters = drivePosition.getValueAsDouble() * WHEEL_RADIUS;
    super.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble() * WHEEL_RADIUS;
    super.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    super.driveCurrentAmps = driveCurrent.getValueAsDouble();

    super.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    super.encoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    super.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    super.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    super.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    super.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    super.turnCurrentAmps = turnCurrent.getValueAsDouble();

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
    DogLog.log(
        "Drive/Module " + constants.prefix() + "/DriveMotor/DrivePositionMeters",
        super.drivePositionMeters);
    DogLog.log(
        "Drive/Module " + constants.prefix() + "/DriveMotor/DriveVelocityMetersPerSec",
        super.driveVelocityMetersPerSec);

    MotorLog.log("Drive/Module " + constants.prefix() + "/TurnMotor", turnTalon);
    DogLog.log("Drive/Module " + constants.prefix() + "/TurnMotor/Connected", super.turnConnected);

    DogLog.log("Drive/Module " + constants.prefix() + "/Encoder/Connected", super.encoderConnected);
    DogLog.log(
        "Drive/Module " + constants.prefix() + "/Encoder/AbsolutePosition",
        super.turnAbsolutePosition);

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
      driveTalon.setControl(drivePID.withVelocity(metersPerSecond));
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
