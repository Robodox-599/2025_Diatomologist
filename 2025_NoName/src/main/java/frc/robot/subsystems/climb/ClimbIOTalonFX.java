package frc.robot.subsystems.climb;

import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.gearRatio;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.climb.ClimbConstants.ClimbStates;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class ClimbIOTalonFX extends ClimbIO {

  private final TalonFX climbMotor;
  private final TalonFX rollersMotor;
  private final BangBangController bangBangController;

   private final StatusSignal<Angle> climbPosition;
   private final StatusSignal<AngularVelocity> climbVelocity;
   private final StatusSignal<Voltage> climbAppliedVolts;
   private final StatusSignal<Current> climbCurrent;
   private final StatusSignal<Temperature> climbTemperature;

   private final StatusSignal<AngularVelocity> rollersVelocity;
   private final StatusSignal<Voltage> rollersAppliedVolts;
   private final StatusSignal<Current> rollersCurrent;
   private final StatusSignal<Temperature> rollersTemperature;
   private final StatusSignal<Current> rollersStatorCurrent;

  public ClimbIOTalonFX() {
    climbMotor = new TalonFX(ClimbConstants.climbMotorID, ClimbConstants.climbMotorCANbus);
    rollersMotor = new TalonFX(ClimbConstants.rollersMotorID, ClimbConstants.rollersMotorCANbus);
    bangBangController = new BangBangController();
    TalonFXConfiguration climbConfig = new TalonFXConfiguration();
    TalonFXConfiguration rollersConfig = new TalonFXConfiguration();

    climbConfig.Slot0.kP = ClimbConstants.kP;
    climbConfig.Slot0.kI = ClimbConstants.kI;
    climbConfig.Slot0.kD = ClimbConstants.kD;
    climbConfig.Slot0.kV = ClimbConstants.kV;
    climbConfig.Slot0.kS = ClimbConstants.kS;
    
    climbConfig.CurrentLimits.SupplyCurrentLimit = 3;
    climbConfig.CurrentLimits.StatorCurrentLimit = 3;
    climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbConfig.Feedback.RotorToSensorRatio = gearRatio;
    rollersConfig.CurrentLimits.SupplyCurrentLimit = 3;
    rollersConfig.CurrentLimits.StatorCurrentLimit = 3;
    rollersConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollersConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(5, () -> climbMotor.getConfigurator().apply(climbConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> rollersMotor.getConfigurator().apply(rollersConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> climbMotor.setPosition(0.0, 0.25));
    climbMotor.optimizeBusUtilization();
    rollersMotor.optimizeBusUtilization();

    climbPosition = climbMotor.getPosition();
    climbVelocity = climbMotor.getVelocity();
    climbAppliedVolts = climbMotor.getMotorVoltage();
    climbCurrent = climbMotor.getStatorCurrent();
    climbTemperature = climbMotor.getDeviceTemp();
    rollersVelocity = rollersMotor.getVelocity();
    rollersAppliedVolts = rollersMotor.getMotorVoltage();
    rollersCurrent = rollersMotor.getStatorCurrent();
    rollersTemperature = rollersMotor.getDeviceTemp();
    rollersStatorCurrent = rollersMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, climbTemperature, climbVelocity, climbPosition, climbCurrent, climbAppliedVolts,  
        rollersTemperature, rollersVelocity, rollersCurrent, rollersAppliedVolts);
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(
      climbTemperature, climbVelocity, climbPosition, climbCurrent, climbAppliedVolts,  
      rollersTemperature, rollersVelocity, rollersCurrent, rollersAppliedVolts);
      
      super.climbAppliedVolts = climbAppliedVolts.getValueAsDouble();
      super.climbCurrentAmps = climbCurrent.getValueAsDouble();
      super.climbVelocity = climbVelocity.getValueAsDouble();
      super.climbPositionDegrees = climbPosition.getValueAsDouble();
      super.climbTempCelsius = climbTemperature.getValueAsDouble();
      super.rollersAppliedVolts = rollersAppliedVolts.getValueAsDouble();
      super.rollersCurrentAmps = rollersCurrent.getValueAsDouble();
      super.rollersVelocity = rollersVelocity.getValueAsDouble();
      super.rollersTempCelsius = rollersTemperature.getValueAsDouble();
      super.rollersStatorCurrent = rollersStatorCurrent.getValueAsDouble();
      super.isCageDetected = super.rollersStatorCurrent >= 20;
    // super.atSetpoint =
    //     positionError < ClimbConstants.positionToleranceInches
    //         && velocityError < ClimbConstants.velocityToleranceInchesPerSec;
 
    DogLog.log("Climb/IsCageDetected", super.isCageDetected);
    DogLog.log("Climb/StatorCurrentAmps", super.climbCurrentAmps);
    DogLog.log("Climb/AppliedVoltage", super.climbAppliedVolts);
    DogLog.log("Climb/Velocity", super.climbVelocity);
    DogLog.log("Climb/Temperature", super.climbTempCelsius);
    DogLog.log("Climb/CurrentPosition", super.climbPositionDegrees);
    DogLog.log("Climb/TargetPositon", super.targetPositionDegrees);
    DogLog.log("Climb/Rollers/StatorCurrentAmps", super.rollersCurrentAmps);
    DogLog.log("Climb/Rollers/AppliedVoltage", super.rollersAppliedVolts);
    DogLog.log("Climb/Rollers/Velocity", super.rollersVelocity);
    DogLog.log("Climb/Rollers/Temperature", super.rollersTempCelsius);
  }

  @Override
  public void setClimb(ClimbStates state) {
    double targetPositionDegrees =
        MathUtil.clamp(
            SubsystemUtil.climbStateToHeight(state),
            ClimbConstants.climbLowerLimit,
            ClimbConstants.climbUpperLimit);
    climbMotor.setPosition(bangBangController.calculate(targetPositionDegrees));
  }

  @Override
  public void setRollers(double velocity) {
    rollersMotor.set(velocity);
  }

  @Override
  public void setVoltage(double voltage) {
    climbMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void stop() {
    climbMotor.stopMotor();
    rollersMotor.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            climbMotor.setNeutralMode(
                enable
                    ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
                    : com.ctre.phoenix6.signals.NeutralModeValue.Coast));
  }
}
