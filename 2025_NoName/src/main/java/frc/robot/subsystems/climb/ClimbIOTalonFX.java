package frc.robot.subsystems.climb;

import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.gearRatio;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
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
import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.PhoenixUtil;

public class ClimbIOTalonFX extends ClimbIO {

  private final TalonFX climbMotor;
  private final TalonFX rollersMotor;
  // private final BangBangController bangBangController;
  // private final DigitalInput cageLimitSwitch;
  private final DigitalInput deployLimitSwitch;
  // private final DigitalInput climbLimitSwitch;
  private final Servo flapServo1;
  private final Servo flapServo2;
  private final Servo rampServo1;
  private final Servo rampServo2;
  Debouncer deployDebouncer = new Debouncer(0.1);
  Debouncer cageDetectDebouncer = new Debouncer(0.1);
  Debouncer climbDebouncer = new Debouncer(0.1);
  Debouncer flapDeployDebouncer = new Debouncer(0.75);
  Debouncer rampDeployDebouncer = new Debouncer(0.75);

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
    // bangBangController = new BangBangController();

    // cageLimitSwitch = new DigitalInput(ClimbConstants.rollersLimitSwitchDioPort);
    deployLimitSwitch = new DigitalInput(ClimbConstants.deployLimitSwitchDioPort);
    // climbLimitSwitch = new DigitalInput(ClimbConstants.climbLimitSwitchDioPort);

    rampServo1 = new Servo(ClimbConstants.rampServoPWMPort1);
    rampServo2 = new Servo(ClimbConstants.rampServoPWMPort2);
    flapServo1 = new Servo(ClimbConstants.flapServoPWMPort1);
    flapServo2 = new Servo(ClimbConstants.flapServoPWMPort2);

    TalonFXConfiguration climbConfig = new TalonFXConfiguration();
    TalonFXConfiguration rollersConfig = new TalonFXConfiguration();

    // climbConfig.Slot0.kP = ClimbConstants.kP;
    // climbConfig.Slot0.kI = ClimbConstants.kI;
    // climbConfig.Slot0.kD = ClimbConstants.kD;
    // climbConfig.Slot0.kV = ClimbConstants.kV;
    // climbConfig.Slot0.kS = ClimbConstants.kS;

    climbConfig.CurrentLimits.SupplyCurrentLimit = 120;
    climbConfig.CurrentLimits.StatorCurrentLimit = 120;
    climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbConfig.Feedback.RotorToSensorRatio = gearRatio;
    climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollersConfig.CurrentLimits.SupplyCurrentLimit = 40;
    rollersConfig.CurrentLimits.StatorCurrentLimit = 60;
    rollersConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollersConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // bangBangController.setTolerance(3);

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

    cageDetectDebouncer.setDebounceType(Debouncer.DebounceType.kRising);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        climbTemperature,
        climbVelocity,
        climbPosition,
        climbCurrent,
        climbAppliedVolts,
        rollersTemperature,
        rollersVelocity,
        rollersCurrent,
        rollersAppliedVolts);

    // zeroEncoder();
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        climbTemperature,
        climbVelocity,
        climbPosition,
        climbCurrent,
        climbAppliedVolts,
        rollersTemperature,
        rollersVelocity,
        rollersCurrent,
        rollersAppliedVolts);

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

    super.isFlapsReleased =
        flapDeployDebouncer.calculate(flapServo1.getAngle() > 170 && flapServo2.getAngle() > 170);
    super.isRampReleased =
        rampDeployDebouncer.calculate(rampServo1.getAngle() > 125 && rampServo2.getAngle() < 25);

    super.isClimbDeployed = deployDebouncer.calculate(!deployLimitSwitch.get());
    super.isCageDetected = cageDetectDebouncer.calculate(super.rollersStatorCurrent >= 20);
    // super.isClimbed = climbDebouncer.calculate(climbLimitSwitch.get());

    // super.atSetpoint = bangBangController.atSetpoint();
    // super.atSetpoint =
    //     positionError < ClimbConstants.positionToleranceInches
    //         && velocityError < ClimbConstants.velocityToleranceInchesPerSec;

    DogLog.log("Climb/IsClimbDeployed", super.isClimbDeployed);
    DogLog.log("Climb/IsCageDetected", super.isCageDetected);
    DogLog.log("Climb/IsClimbed", super.isClimbed);

    DogLog.log("Climb/isFlapsReleased", super.isFlapsReleased);
    DogLog.log("Climb/isRampReleased", super.isRampReleased);

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
  public void setRollersVelocity(double velocity) {
    rollersMotor.set(velocity);
  }

  @Override
  public void stallRollers() {
    rollersMotor.setControl(new DutyCycleOut(ClimbConstants.stallRollersVoltage));
  }

  @Override
  public void setClimbVoltage(double voltage) {
    climbMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void releaseFlapServos() {
    flapServo1.setAngle(180);
    flapServo2.setAngle(180);
  }

  @Override
  public void releaseRampServos() {
    rampServo1.setAngle(135);
    rampServo2.setAngle(15);
  }

  // @Override
  // public void zeroEncoder() {
  //   climbMotor.setPosition(0);
  // }

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
