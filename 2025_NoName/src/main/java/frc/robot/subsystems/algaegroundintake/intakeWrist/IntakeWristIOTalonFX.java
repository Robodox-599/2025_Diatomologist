package frc.robot.subsystems.algaegroundintake.intakewrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import frc.robot.util.AlgaeGroundIntakeUtil;
import frc.robot.util.MotorLog;
import frc.robot.util.PhoenixUtil;

public class IntakeWristIOTalonFX extends IntakeWristIO {

  private final TalonFX wristMotor;
  TalonFXConfiguration wristConfig;
  private final MotionMagicVoltage m_request;

  private double passedInPosition;
  private double currentPosition;
  private int wristSlot;

  public IntakeWristIOTalonFX() {

    wristMotor =
        new TalonFX(IntakeWristConstants.wristMotorID, IntakeWristConstants.wristMotorCANBus);
    wristConfig = new TalonFXConfiguration();
    m_request = new MotionMagicVoltage(0);

    var motionMagicConfigs = wristConfig.MotionMagic;
    // I don't really know what values to put here :(
    motionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
    motionMagicConfigs.MotionMagicAcceleration = 0.0;

    wristConfig.Slot0.kP = IntakeWristConstants.wristExtendKP;
    wristConfig.Slot0.kI = IntakeWristConstants.wristExtendKI;
    wristConfig.Slot0.kD = IntakeWristConstants.wristExtendKD;
    wristConfig.Slot0.kV = IntakeWristConstants.wristExtendKV;

    wristConfig.Slot1.kP = IntakeWristConstants.wristRetractKP;
    wristConfig.Slot1.kI = IntakeWristConstants.wristRetractKI;
    wristConfig.Slot1.kD = IntakeWristConstants.wristRetractKD;
    wristConfig.Slot1.kV = IntakeWristConstants.wristRetractKV;

    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = IntakeWristConstants.EnableCurrentLimit;
    wristConfig.CurrentLimits.SupplyCurrentLimit = IntakeWristConstants.ContinousCurrentLimit;
    wristConfig.CurrentLimits.SupplyCurrentLowerLimit = IntakeWristConstants.PeakCurrentDuration;
    wristConfig.CurrentLimits.SupplyCurrentLowerTime = IntakeWristConstants.PeakCurrentDuration;

    wristConfig.Feedback.RotorToSensorRatio = IntakeWristConstants.gearRatio;

    wristMotor.optimizeBusUtilization();
    wristMotor.getConfigurator().apply(wristConfig);

    PhoenixUtil.tryUntilOk(5, () -> wristMotor.getConfigurator().apply(wristConfig, 0.25));
  }

  @Override
  public void updateInputs() {
    super.appliedVolts = wristMotor.getMotorVoltage().getValueAsDouble();
    super.currentAmps = wristMotor.getSupplyCurrent().getValueAsDouble();
    super.velocity = wristMotor.getVelocity().getValueAsDouble();
    super.tempCelsius = wristMotor.getDeviceTemp().getValueAsDouble();
    super.position = wristMotor.getPosition().getValueAsDouble();
    super.targetPosition = targetPosition;
    super.currentPosition = currentPosition;

    MotorLog.log("IntakeWrist", wristMotor);

    DogLog.log("IntakeWrist/TargetPosition", passedInPosition);
    DogLog.log("IntakeWrist/CurrentPosition", currentPosition);
    DogLog.log("IntakeWrist/Position", wristMotor.getPosition().getValueAsDouble());
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
  public void setState(IntakeWristConstants.AlgaeStates state) {
    double position =
        MathUtil.clamp(
            AlgaeGroundIntakeUtil.stateToSetpoint(state),
            IntakeWristConstants.intakeWristLowerLimit,
            IntakeWristConstants.intakeWristUpperLimit);

    switch (state) {
      case STOW:
        position = IntakeWristConstants.setpoints[2];
        break;
      case DEPLOYED:
        position = IntakeWristConstants.setpoints[0];
      default:
        position = IntakeWristConstants.setpoints[1]; // STOW
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
