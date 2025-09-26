package frc.robot.subsystems.endefector.endefectorwrist;

import static frc.robot.subsystems.endefector.endefectorwrist.WristConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class WristIOTalonFX extends WristIO {

  private final TalonFX wristMotor;
  TalonFXConfiguration wristConfig;
  private final MotionMagicVoltage m_request;

  private final CANcoder cancoder;

  // Inputs from turn motor
  private final StatusSignal<Angle> absolutePosition;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temperature;

  public WristIOTalonFX() {

    wristMotor = new TalonFX(wristMotorID, wristMotorCANBus);
    wristConfig = new TalonFXConfiguration();
    m_request =
        new MotionMagicVoltage(SubsystemUtil.wristStateToSetpoint(WristStates.POSITION_PREPARED))
            .withSlot(0)
            .withEnableFOC(true);

    cancoder = new CANcoder(cancoderID, wristMotorCANBus);
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    wristConfig.MotionMagic.MotionMagicCruiseVelocity = (12 - realkG - realkS) / realkV * 2;

    wristConfig.MotionMagic.MotionMagicAcceleration = (((12 - realkG - realkS) / realkV) * 8);

    wristConfig.Slot0.kP = realkP;
    wristConfig.Slot0.kI = realkI;
    wristConfig.Slot0.kD = realkD;
    wristConfig.Slot0.kV = realkV;
    wristConfig.Slot0.kS = realkS;
    wristConfig.Slot0.kG = realkG;

    wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wristConfig.CurrentLimits.SupplyCurrentLimit = 40;
    wristConfig.CurrentLimits.StatorCurrentLimit = 60;

    // wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.1;
    // wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.52;

    wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wristConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
    wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristConfig.Feedback.RotorToSensorRatio = gearRatio;
    wristConfig.ClosedLoopGeneral.ContinuousWrap = false;

    cancoderConfig.MagnetSensor.MagnetOffset = cancoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.4;
    PhoenixUtil.tryUntilOk(10, () -> wristMotor.getConfigurator().apply(wristConfig, 1));
    PhoenixUtil.tryUntilOk(10, () -> cancoder.getConfigurator().apply(cancoderConfig, 1));
    wristMotor.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
    absolutePosition = cancoder.getAbsolutePosition();
    position = wristMotor.getPosition();
    velocity = wristMotor.getVelocity();
    appliedVolts = wristMotor.getMotorVoltage();
    current = wristMotor.getStatorCurrent();
    temperature = wristMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, absolutePosition, temperature, velocity, position, current, appliedVolts);
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        absolutePosition, temperature, velocity, position, current, appliedVolts);
    super.appliedVolts = appliedVolts.getValueAsDouble();
    super.currentAmps = current.getValueAsDouble();
    super.velocity = velocity.getValueAsDouble();
    super.currentPosition = position.getValueAsDouble();
    super.tempCelsius = temperature.getValueAsDouble();
    super.atSetpoint =
        Math.abs(super.currentPosition - super.targetPosition) < wristPositionTolerance;

    DogLog.log("Wrist/AppliedVoltage", super.appliedVolts);
    DogLog.log("Wrist/CurrentAmps", super.currentAmps);
    DogLog.log("Wrist/Velocity", super.velocity);
    DogLog.log("Wrist/Temperature", super.tempCelsius);
    DogLog.log("Wrist/CurrentPosition", super.currentPosition);
    DogLog.log("Wrist/WristAtSetpoint", super.atSetpoint);
    DogLog.log("Wrist/AbsolutePosition", absolutePosition.getValueAsDouble());
    DogLog.log("Wrist/TargetPosition", targetPosition);
  }

  @Override
  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
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
  public void setAngle(WristStates state) {
    double position =
        MathUtil.clamp(SubsystemUtil.wristStateToSetpoint(state), wristMinAngle, wristMaxAngle);
    super.targetPosition = position;
    m_request.withPosition(position);
    wristMotor.setControl(m_request);
  }
}
