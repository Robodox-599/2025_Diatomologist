package frc.robot.subsystems.endefector.endefectorwrist;

import static frc.robot.subsystems.endefector.endefectorwrist.WristConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.util.PhoenixUtil;

public class WristIOTalonFX extends WristIO {

  private final TalonFX wristMotor;
  TalonFXConfiguration wristConfig;
  private final MotionMagicVoltage m_request;

  private final CANcoder cancoder;

  private double passedInPosition;
  private double currentPosition;

  public WristIOTalonFX() {

    wristMotor = new TalonFX(wristMotorID, wristMotorCANBus);
    wristConfig = new TalonFXConfiguration();
    m_request = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

    cancoder = new CANcoder(cancoderID, wristMotorCANBus);
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    var motionMagicConfigs = wristConfig.MotionMagic;
    // I don't really know what values to put here :(
    motionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
    motionMagicConfigs.MotionMagicAcceleration = 0.0;

    wristConfig.Slot0.kP = realkP;
    wristConfig.Slot0.kI = realkI;
    wristConfig.Slot0.kD = realkD;
    wristConfig.Slot0.kV = realkV;
    wristConfig.Slot0.kS = realkS;
    wristConfig.Slot0.kG = realkG;
    wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    wristConfig.CurrentLimits.SupplyCurrentLimit = 40;
    wristConfig.CurrentLimits.StatorCurrentLimit = 60;

    wristConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
    wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    wristConfig.Feedback.RotorToSensorRatio = gearRatio;

    cancoderConfig.MagnetSensor.MagnetOffset = cancoderOffset;

    PhoenixUtil.tryUntilOk(5, () -> wristMotor.getConfigurator().apply(wristConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));
    wristMotor.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs() {
    super.appliedVolts = wristMotor.getMotorVoltage().getValueAsDouble();
    super.currentAmps = wristMotor.getSupplyCurrent().getValueAsDouble();
    super.velocity = wristMotor.getVelocity().getValueAsDouble();
    super.tempCelsius = wristMotor.getDeviceTemp().getValueAsDouble();
    super.currentPositionDegrees = wristMotor.getPosition().getValueAsDouble();
    super.targetPosition = this.targetPosition;
    super.atSetpoint =
        Math.abs(super.currentPositionDegrees - this.targetPosition) < wristPositionTolerance;

    DogLog.log("Wrist/AppliedVoltage", super.appliedVolts);
    DogLog.log("Wrist/CurrentAmps", super.currentAmps);
    DogLog.log("Wrist/Velocity", super.velocity);
    DogLog.log("Wrist/Temperature", super.tempCelsius);
    DogLog.log("Wrist/CurrentPosition", super.currentPositionDegrees);
    DogLog.log("Wrist/AtSetpoint", super.atSetpoint);
  }

  @Override
  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  @Override
  public void goToPose(double position) {
    passedInPosition = position;
    m_request.withPosition(passedInPosition);
    wristMotor.setControl(m_request);
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
    double position =
        MathUtil.clamp(WristConstants.setpoints[state.getIndex()], wristMinAngle, wristMaxAngle);
    m_request.Position = position;
    wristMotor.setControl(m_request);
  }

  @Override
  public double getCurrentPosition() {
    return this.currentPosition;
  }
}
