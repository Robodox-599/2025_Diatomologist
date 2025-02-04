package frc.robot.subsystems.endefector.wrist;

import static frc.robot.subsystems.endefector.wrist.WristConstants.*;
import static frc.robot.util.WristUtil.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.endefector.wrist.WristConstants.WristStates;
import frc.robot.util.MotorLog;
import frc.robot.util.PhoenixUtil;

// import edu.wpi.first.math.MathUtil;

public class WristIOTalonFX extends WristIO {

  private final TalonFX wristMotor;
  TalonFXConfiguration wristConfig;
  private final MotionMagicVoltage m_request;

  private final CANcoder cancoder;

  private double passedInPosition;
  private double currentPosition;
  private int wristSlot;

  public WristIOTalonFX() {

    wristMotor = new TalonFX(wristMotorID, wristMotorCANBus);
    wristConfig = new TalonFXConfiguration();
    m_request = new MotionMagicVoltage(0);

    cancoder = new CANcoder(cancoderID, wristMotorCANBus);
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    var motionMagicConfigs = wristConfig.MotionMagic;
    // I don't really know what values to put here :(
    motionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
    motionMagicConfigs.MotionMagicAcceleration = 0.0;

    wristConfig.Slot0.kP = realExtendkP;
    wristConfig.Slot0.kI = realExtendkI;
    wristConfig.Slot0.kD = realExtendkD;
    wristConfig.Slot0.kV = realExtendkV;

    wristConfig.Slot1.kP = realRetractkP;
    wristConfig.Slot1.kI = realRetractkI;
    wristConfig.Slot1.kD = realRetractkD;
    wristConfig.Slot1.kV = realRetractkV;

    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    wristConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    wristConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    wristConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    wristConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
    wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    wristConfig.Feedback.RotorToSensorRatio = gearRatio;

    cancoderConfig.MagnetSensor.MagnetOffset = cancoderOffset;

    wristMotor.optimizeBusUtilization();
    wristMotor.getConfigurator().apply(wristConfig);

    PhoenixUtil.tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));
  }

  @Override
  public void updateInputs() {
    super.appliedVolts = wristMotor.getMotorVoltage().getValueAsDouble();
    super.currentAmps = wristMotor.getSupplyCurrent().getValueAsDouble();
    super.velocity = wristMotor.getVelocity().getValueAsDouble();
    super.tempCelsius = wristMotor.getDeviceTemp().getValueAsDouble();
    super.position = wristMotor.getPosition().getValueAsDouble();
    super.targetPosition = this.targetPosition;
    this.currentPosition = wristMotor.getPosition().getValueAsDouble();
    super.currentPosition = this.currentPosition;
    super.atSetpoint =
        Math.abs(super.currentPosition - this.targetPosition) < wristPositionTolerance;
    MotorLog.log("Wrist", wristMotor);

    DogLog.log("Wrist/TargetPosition", passedInPosition);
    DogLog.log("Wrist/CurrentPosition", currentPosition);
  }

  @Override
  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  @Override
  public void goToPose(double position) {
    passedInPosition = position;

    if (passedInPosition > currentPosition) {
      wristSlot = 0;
    } else {
      wristSlot = 1;
    }

    m_request.withSlot(wristSlot);
    wristMotor.setControl(m_request);
  }

  @Override
  public double getPose() {
    return wristMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void stop() {
    wristMotor.stopMotor();
  }

  @Override
  public void setBrake(boolean brake) {
    wristMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setState(WristStates state) {
    double position = MathUtil.clamp(stateToHeight(state), wristLowerLimit, wristUpperLimit);

    switch (state) {
      case STOW:
        position = setpoints[0];
        break;
      case SCORING:
        position = setpoints[1];
        break;
      case OVERRIDE:
        position = setpoints[2];
        break;
      case GROUNDINTAKE:
        position = setpoints[3];
        break;
      case STATIONINTAKE:
        position = setpoints[3];
        break;
      case CLIMB:
        position = setpoints[3];
        break;
      default:
        position = setpoints[0]; // STOW
        break;
    }
    if (passedInPosition > currentPosition) {
      wristSlot = 0;
    } else {
      wristSlot = 1;
    }

    m_request.withSlot(0);
    m_request.Position = position;
    wristMotor.setControl(m_request);
  }
}
