package frc.robot.subsystems.endefector.endefectorrollers;

import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.EndefectorRollerStates;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class RollersIOTalonFX extends RollersIO {
  private final TalonFX endefectorRollersMotor;
  private final TalonFX rampRollersMotor;
  TalonFXConfiguration endefectorRollersConfig;
  TalonFXConfiguration rampRollersConfig;
  Debouncer rampCoralDebouncer = new Debouncer(rampCoralDebounce);
  Debouncer coralIntakeDebouncer = new Debouncer(coralIntakeDebounce);
  Debouncer algaeGroundIntakeDebouncer = new Debouncer(algaeGroundIntakeDebounce);
  Debouncer algaeReefIntakeDebouncer = new Debouncer(algaeReefIntakeDebounce);
  Debouncer coralTroughScoreDebouncer = new Debouncer(coralTroughScoreDebounce);
  Debouncer coralBranchScoreDebouncer = new Debouncer(coralBranchScoreDebounce);
  Debouncer algaeScoreDebouncer = new Debouncer(algaeScoreDebounce);
  private DigitalInput rampBeamBreak;
  private DigitalInput transitionBeamBreak;
  private DigitalInput endefectorBeamBreak;
  private AsynchronousInterrupt transitionBeamBreakInterrupt;

  private final StatusSignal<AngularVelocity> endefectorRollersVelocity;
  private final StatusSignal<Voltage> endefectorRollersAppliedVolts;
  private final StatusSignal<Current> endefectorRollersStatorCurrent;
  private final StatusSignal<Current> endefectorRollersSupplyCurrent;
  private final StatusSignal<Temperature> endefectorRollersTemperature;
  private final StatusSignal<Angle> endefectorRollersPosition;

  private final StatusSignal<AngularVelocity> rampRollersVelocity;
  private final StatusSignal<Voltage> rampRollersAppliedVolts;
  private final StatusSignal<Current> rampRollersStatorCurrent;
  private final StatusSignal<Current> rampRollersSupplyCurrent;
  private final StatusSignal<Temperature> rampRollersTemperature;
  private final StatusSignal<Angle> rampRollersPosition;

  public RollersIOTalonFX() {
    endefectorRollersMotor = new TalonFX(endefectorRollersMotorID, endefectorRollersMotorCANBus);
    rampRollersMotor = new TalonFX(rampRollersMotorID, rampRollersMotorCANBus);
    rampBeamBreak = new DigitalInput(RollersConstants.rampBeamBreakPort);
    transitionBeamBreak = new DigitalInput(RollersConstants.transitionBeamBreakPort);
    endefectorBeamBreak = new DigitalInput(RollersConstants.endefectorBeamBreakPort);

    endefectorRollersConfig = new TalonFXConfiguration();
    rampRollersConfig = new TalonFXConfiguration();

    endefectorRollersMotor.setNeutralMode(NeutralModeValue.Brake);
    rampRollersMotor.setNeutralMode(NeutralModeValue.Brake);

    endefectorRollersConfig.Slot0.kP = realP;
    endefectorRollersConfig.Slot0.kI = realI;
    endefectorRollersConfig.Slot0.kD = realD;
    endefectorRollersConfig.Slot0.kS = realkS;
    endefectorRollersConfig.Slot0.kV = realEndefectorkV;

    endefectorRollersConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    endefectorRollersConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    endefectorRollersConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    endefectorRollersConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    rampRollersConfig.Slot0.kP = realP;
    rampRollersConfig.Slot0.kI = realI;
    rampRollersConfig.Slot0.kD = realD;
    rampRollersConfig.Slot0.kS = realkS;
    rampRollersConfig.Slot0.kV = realRampkV;

    rampRollersConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    rampRollersConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    rampRollersConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    rampRollersConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    PhoenixUtil.tryUntilOk(
        10, () -> endefectorRollersMotor.getConfigurator().apply(endefectorRollersConfig, 1));
    PhoenixUtil.tryUntilOk(
        10, () -> rampRollersMotor.getConfigurator().apply(rampRollersConfig, 1));

    rampCoralDebouncer.setDebounceType(DebounceType.kRising);
    coralIntakeDebouncer.setDebounceType(DebounceType.kRising);
    coralTroughScoreDebouncer.setDebounceType(DebounceType.kRising);
    coralBranchScoreDebouncer.setDebounceType(DebounceType.kRising);

    algaeGroundIntakeDebouncer.setDebounceType(DebounceType.kRising);
    algaeReefIntakeDebouncer.setDebounceType(DebounceType.kRising);
    algaeScoreDebouncer.setDebounceType(DebounceType.kFalling);

    endefectorRollersPosition = endefectorRollersMotor.getPosition();
    endefectorRollersVelocity = endefectorRollersMotor.getVelocity();
    endefectorRollersAppliedVolts = endefectorRollersMotor.getMotorVoltage();
    endefectorRollersStatorCurrent = endefectorRollersMotor.getStatorCurrent();
    endefectorRollersTemperature = endefectorRollersMotor.getDeviceTemp();
    endefectorRollersSupplyCurrent = endefectorRollersMotor.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        endefectorRollersPosition,
        endefectorRollersVelocity,
        endefectorRollersTemperature,
        endefectorRollersSupplyCurrent,
        endefectorRollersStatorCurrent,
        endefectorRollersAppliedVolts);

    rampRollersPosition = rampRollersMotor.getPosition();
    rampRollersVelocity = rampRollersMotor.getVelocity();
    rampRollersAppliedVolts = rampRollersMotor.getMotorVoltage();
    rampRollersStatorCurrent = rampRollersMotor.getStatorCurrent();
    rampRollersTemperature = rampRollersMotor.getDeviceTemp();
    rampRollersSupplyCurrent = rampRollersMotor.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rampRollersPosition,
        rampRollersVelocity,
        rampRollersTemperature,
        rampRollersSupplyCurrent,
        rampRollersStatorCurrent,
        rampRollersAppliedVolts);

    endefectorRollersMotor.optimizeBusUtilization();
    rampRollersMotor.optimizeBusUtilization();

    transitionBeamBreakInterrupt =
        new AsynchronousInterrupt(
            transitionBeamBreak,
            (rising, falling) -> {
              if (rising) { // coral -> no coral
                if (!endefectorBeamBreak.get()) { // if coral in endefector
                  setEndefectorHoldCoralPosition();
                }
              } else if (falling) { // no coral -> coral
                setRampHoldCoralPosition();
              }
            });

    transitionBeamBreakInterrupt.enable();
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        endefectorRollersPosition,
        endefectorRollersVelocity,
        endefectorRollersTemperature,
        endefectorRollersSupplyCurrent,
        endefectorRollersStatorCurrent,
        endefectorRollersAppliedVolts);
    BaseStatusSignal.refreshAll(
        rampRollersPosition,
        rampRollersVelocity,
        rampRollersTemperature,
        rampRollersSupplyCurrent,
        rampRollersStatorCurrent,
        rampRollersAppliedVolts);
    super.endefectorRollersPosition = endefectorRollersPosition.getValueAsDouble();
    super.endefectorRollersStatorCurrent = endefectorRollersStatorCurrent.getValueAsDouble();
    super.endefectorRollersSupplyCurrent = endefectorRollersSupplyCurrent.getValueAsDouble();
    super.endefectorRollersVelocity = endefectorRollersVelocity.getValueAsDouble();
    super.endefectorRollersTempCelsius = endefectorRollersTemperature.getValueAsDouble();
    super.endefectorRollersAppliedVolts = endefectorRollersAppliedVolts.getValueAsDouble();

    super.rampRollersPosition = rampRollersPosition.getValueAsDouble();
    super.rampRollersStatorCurrent = rampRollersStatorCurrent.getValueAsDouble();
    super.rampRollersSupplyCurrent = rampRollersSupplyCurrent.getValueAsDouble();
    super.rampRollersVelocity = rampRollersVelocity.getValueAsDouble();
    super.rampRollersTempCelsius = rampRollersTemperature.getValueAsDouble();
    super.rampRollersAppliedVolts = rampRollersAppliedVolts.getValueAsDouble();

    DogLog.log("Rollers/Endefector/StatorCurrentAmps", super.endefectorRollersStatorCurrent);
    DogLog.log("Rollers/Endefector/SupplyCurrentAmps", super.endefectorRollersSupplyCurrent);
    DogLog.log("Rollers/Endefector/Position", super.endefectorRollersPosition);
    DogLog.log("Rollers/Endefector/Velocity", super.endefectorRollersVelocity);
    DogLog.log("Rollers/Endefector/AppliedVoltage", super.endefectorRollersAppliedVolts);
    DogLog.log("Rollers/Endefector/TempCelcius", super.endefectorRollersTempCelsius);

    DogLog.log("Rollers/Ramp/StatorCurrentAmps", super.rampRollersStatorCurrent);
    DogLog.log("Rollers/Ramp/SupplyCurrentAmps", super.rampRollersSupplyCurrent);
    DogLog.log("Rollers/Ramp/Position", super.rampRollersPosition);
    DogLog.log("Rollers/Ramp/Velocity", super.rampRollersVelocity);
    DogLog.log("Rollers/Ramp/AppliedVoltage", super.rampRollersAppliedVolts);
    DogLog.log("Rollers/Ramp/TempCelcius", super.rampRollersTempCelsius);

    super.isCoralInRamp = rampCoralDebouncer.calculate(!rampBeamBreak.get());
    super.isCoralIntakedInEndefector = coralIntakeDebouncer.calculate(!endefectorBeamBreak.get());
    super.isGroundAlgaeIntaked =
        algaeGroundIntakeDebouncer.calculate(
            super.endefectorRollersStatorCurrent >= algaeStallStatorCurrentAmps);
    super.isReefAlgaeIntaked =
        algaeReefIntakeDebouncer.calculate(
            super.endefectorRollersStatorCurrent >= algaeStallStatorCurrentAmps);
    super.isCoralTroughScored = coralTroughScoreDebouncer.calculate(endefectorBeamBreak.get());
    super.isCoralBranchScored = coralBranchScoreDebouncer.calculate(endefectorBeamBreak.get());
    super.isAlgaeScored =
        algaeScoreDebouncer.calculate(
            !(super.endefectorRollersStatorCurrent >= algaeStallStatorCurrentAmps));

    DogLog.log("Rollers/CoralInRamp", super.isCoralInRamp);
    DogLog.log("Rollers/CoralIntakedInEndefector", super.isCoralIntakedInEndefector);
    DogLog.log("Rollers/GroundAlgaeIntaked", super.isGroundAlgaeIntaked);
    DogLog.log("Rollers/ReefAlgaeIntaked", super.isReefAlgaeIntaked);
    DogLog.log("Rollers/CoralTroughScored", super.isCoralTroughScored);
    DogLog.log("Rollers/CoralBranchScored", super.isCoralBranchScored);
    DogLog.log("Rollers/AlgaeScored", super.isAlgaeScored);

    DogLog.log("Rollers/Endefector/endefectorHoldCoralPosition", super.endefectorHoldCoralPosition);
    DogLog.log("Rollers/Ramp/rampHoldCoralPosition", super.rampHoldCoralPosition);

    DogLog.log("Rollers/RampBeamBreak", rampBeamBreak.get());
    DogLog.log("Rollers/EndefectorBeamBreak", endefectorBeamBreak.get());
    DogLog.log("Rollers/TransitionBeamBreak", transitionBeamBreak.get());
  }

  @Override
  public void stop() {
    setEndefectorVelocity(EndefectorRollerStates.STOPPED);
    setRampVelocity(0.0);
  }

  @Override
  public void setEndefectorVelocity(EndefectorRollerStates state) {
    endefectorRollersMotor.set(SubsystemUtil.endefectorRollersStateToVelocity(state));
  }

  @Override
  public void setRampVelocity(double speed) {
    rampRollersMotor.set(speed);
  }

  @Override
  public void holdAlgae() {
    endefectorRollersMotor.setControl(new DutyCycleOut(rollersDutyCycleOutHoldAlgae));
  }

  @Override
  public void setRampHoldCoralPosition() {
    super.rampHoldCoralPosition = rampRollersMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setEndefectorHoldCoralPosition() {
    super.endefectorHoldCoralPosition = endefectorRollersMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void rampHoldCoral() {
    rampRollersMotor.setControl(new PositionDutyCycle(super.rampHoldCoralPosition));
  }

  @Override
  public void endefectorHoldCoral() {
    endefectorRollersMotor.setControl(new PositionDutyCycle(super.endefectorHoldCoralPosition));
  }

  @Override
  public void resetRollersPosition() {
    endefectorRollersMotor.setPosition(0.0);
    rampRollersMotor.setPosition(0.0);
  }
}
