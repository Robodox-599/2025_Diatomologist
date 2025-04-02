package frc.robot.subsystems.endefector.endefectorrollers;

import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.PhoenixUtil;

public class RollersIOTalonFX extends RollersIO {

  private final TalonFX rollersMotor;
  TalonFXConfiguration rollersConfig;
  private TorqueCurrentFOC torqueCurrent;
  Debouncer algaeStallDebouncer = new Debouncer(0.5);
  Debouncer coralBeamBreakDebouncer = new Debouncer(0.3);
  // private Timer beamBreakTimer = new Timer();
  // private Timer PETimer = new Timer();
  private DigitalInput m_BeamBreak2;
  private DigitalInput PESensor;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;

  private double desiredVelocity;

  public RollersIOTalonFX() {
    rollersMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
    m_BeamBreak2 = new DigitalInput(RollersConstants.beakBreakPort);
    PESensor = new DigitalInput(RollersConstants.PESensorPort);
    torqueCurrent = new TorqueCurrentFOC(65);
    rollersConfig = new TalonFXConfiguration();

    rollersMotor.setNeutralMode(NeutralModeValue.Brake);

    rollersConfig.Slot0.kP = realP;
    rollersConfig.Slot0.kI = realI;
    rollersConfig.Slot0.kD = realD;
    rollersConfig.Slot0.kS = realS;
    rollersConfig.Slot0.kV = realV;

    rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    rollersConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    rollersConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    rollersConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    PhoenixUtil.tryUntilOk(10, () -> rollersMotor.getConfigurator().apply(rollersConfig, 1));
    rollersMotor.optimizeBusUtilization();
    position = rollersMotor.getPosition();
    velocity = rollersMotor.getVelocity();
    appliedVolts = rollersMotor.getMotorVoltage();
    statorCurrent = rollersMotor.getStatorCurrent();
    temperature = rollersMotor.getDeviceTemp();
    supplyCurrent = rollersMotor.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, temperature, supplyCurrent, position, statorCurrent, appliedVolts);
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        velocity, temperature, position, statorCurrent, supplyCurrent, appliedVolts);
    super.appliedVolts = appliedVolts.getValueAsDouble();
    super.statorCurrentAmps = statorCurrent.getValueAsDouble();
    super.supplyCurrentAmps = supplyCurrent.getValueAsDouble();

    super.velocity = velocity.getValueAsDouble();
    super.tempCelsius = temperature.getValueAsDouble();
    super.desiredVelocity = desiredVelocity;
    super.isAlgaeDetected = algaeStallDebouncer.calculate(super.statorCurrentAmps >= 20);
    super.isCoralDetected = coralBeamBreakDebouncer.calculate(!m_BeamBreak2.get());
    // if (m_BeamBreak2.get()) {
    //   beamBreakTimer.restart();
    // }
    // if (PESensor.get()) {
    //   PETimer.restart();
    // }
    DogLog.log("Rollers/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Rollers/SupplyCurrentAmps", super.supplyCurrentAmps);

    DogLog.log("Rollers/Velocity", super.velocity);
    DogLog.log("Rollers/AppliedVoltage", super.appliedVolts);
    DogLog.log("Rollers/TempCelcius", super.tempCelsius);
    DogLog.log("Rollers/VelocitySetpoint", desiredVelocity);
    DogLog.log("Rollers/State", super.currentState);
    DogLog.log("Rollers/AlgaeDetected", super.isAlgaeDetected);
    DogLog.log("Rollers/CoralDetected", super.isCoralDetected);
    DogLog.log("Rollers/BeamBreak", m_BeamBreak2.get());
    // DogLog.log("Rollers/AlgaeDetected", this.isAlgaeDetected());
    DogLog.log("Rollers/CANRangeDistance", super.canrangeDistance);
  }

  @Override
  public void stop() {
    setVelocity(0);
  }

  @Override
  public void setVelocity(double velocity) {
    desiredVelocity = velocity;
    rollersMotor.set(velocity);
  }

  @Override
  public void adjustCoralAfterStationIntake() {
    rollersMotor.setControl(
        new PositionVoltage(
            rollersMotor.getPosition().getValueAsDouble()
                + RollersConstants.rotationsToMoveAfterDetectingCoral));
  }

  @Override
  public void holdAlgae() {
    rollersMotor.setControl(new DutyCycleOut(rollersDutyCycleOutHoldAlgae));
  }

  @Override
  public void setState(RollersConstants.EndefectorRollerStates state) {
    super.currentState = state;
    switch (state) {
      case SCORECORAL:
        setVelocity(rollersCoralScoreSpeed);
        break;
      case SCOREALGAE:
        setVelocity(rollersAlgaeScoreSpeed);
        break;
      case CORALSTATIONINTAKE:
        setVelocity(rollersCoralStationIntakeSpeed);
        break;
      case ALGAEINTAKE:
        setVelocity(rollersAlgaeIntakeSpeed);
        break;
      case ADJUSTCORALAFTERSTATIONINTAKE:
        adjustCoralAfterStationIntake();
        break;
      case HOLDALGAE:
        holdAlgae();
        break;
      case STOP:
        setVelocity(0);
        break;
      case EJECT:
        setVelocity(rollersEjectSpeed);
        break;
      default:
        setVelocity(0);
        break;
    }
  }

  @Override
  public boolean isCoralDetected() {
    return super.isCoralDetected;
  }

  // @Override
  // public boolean isAlgaeDetected() {
  //   return (PETimer.get() >= 0.1);
  // }

  @Override
  public boolean isAlgaeDetected() {
    return super.isAlgaeDetected;
  }

  // @Override
  // public boolean isRollersStalling() {
  //   DogLog.log("Rollers/StatorCurrentAmps", super.currentAmps);
  //   return (super.currentAmps >= 20);
  // }

  // @Override
  // public void setVoltage(double voltage) {
  //   rollersMotor.setVoltage(voltage);
  // }

  // @Override
  // public void setSpeed(double speed) {
  //   rollersMotor.set(speed);
  // }

  // @Override
  // public void setBrake(boolean brake) {
  //   rollersMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  // }

}
