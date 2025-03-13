package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class ElevatorIOTalonFX extends ElevatorIO {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  private final DigitalInput limitSwitch1;
  private final DigitalInput limitSwitch2;
  private ElevatorConstants.ElevatorStates currentState = ElevatorConstants.ElevatorStates.STOW;
  private final MotionMagicVoltage motionMagicRequest;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temperature;

  public ElevatorIOTalonFX() {
    leaderMotor = new TalonFX(ElevatorConstants.leaderMotorID, ElevatorConstants.leaderMotorCANbus);
    followerMotor =
        new TalonFX(ElevatorConstants.followerMotorID, ElevatorConstants.followerMotorCANbus);
    /*  This tells the motor encoder where 0 inches is*/
    limitSwitch1 = new DigitalInput(ElevatorConstants.limitSwitchDioPort1);
    limitSwitch2 = new DigitalInput(ElevatorConstants.limitSwitchDioPort2);

    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), true));

    motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.maxVelocityRotsPerSec;
    config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.maxAccelerationRotationsPerSecSQ;

    config.Slot0.kP = ElevatorConstants.kP;
    config.Slot0.kI = ElevatorConstants.kI;
    config.Slot0.kD = ElevatorConstants.kD;
    config.Slot0.kV = ElevatorConstants.kV;
    config.Slot0.kS = ElevatorConstants.kS;
    config.Slot0.kG = ElevatorConstants.kG;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.statorCurrentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.supplyCurrentLimitAmps;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    PhoenixUtil.tryUntilOk(10, () -> leaderMotor.getConfigurator().apply(config, 1));
    PhoenixUtil.tryUntilOk(10, () -> followerMotor.getConfigurator().apply(config, 1));

    leaderMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();

    position = leaderMotor.getPosition();
    velocity = leaderMotor.getVelocity();
    appliedVolts = leaderMotor.getMotorVoltage();
    current = leaderMotor.getStatorCurrent();
    temperature = leaderMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, temperature, position, current, appliedVolts);

    zeroEncoder();
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(velocity, temperature, position, current, appliedVolts);
    super.positionInches = position.getValueAsDouble() * ElevatorConstants.inchesPerRev;
    super.velocityInchesPerSec = velocity.getValueAsDouble() * ElevatorConstants.inchesPerRev;
    super.appliedVolts = appliedVolts.getValueAsDouble();
    super.currentAmps = current.getValueAsDouble();
    super.targetPositionInches = motionMagicRequest.Position * ElevatorConstants.inchesPerRev;
    super.state = currentState;

    /* Determines if the elevator is at a setpoint */
    double positionError = Math.abs(super.targetPositionInches - super.positionInches);
    super.atSetpoint = positionError < ElevatorConstants.positionToleranceInches;
    /*Leader motor */
    DogLog.log("Elevator/ElevatorLeader/StatorCurrentAmps", super.currentAmps);
    DogLog.log("Elevator/ElevatorLeader/AppliedVoltage", super.appliedVolts);
    DogLog.log("Elevator/ElevatorLeader/TempCelcius", super.tempCelsius);

    // Follower motor
    DogLog.log("Elevator/ElevatorFollower/StatorCurrentAmps", super.currentAmps);
    DogLog.log("Elevator/ElevatorFollower/AppliedVoltage", super.appliedVolts);
    DogLog.log("Elevator/ElevatorFollower/TempCelcius", super.tempCelsius);

    /* Log all super */
    DogLog.log("Elevator/TargetPositionInches", super.targetPositionInches);
    DogLog.log("Elevator/ElevatorAtSetpoint", super.atSetpoint);
    DogLog.log("Elevator/State", super.state.toString());
    DogLog.log("Elevator/LimitSwitchValue", super.limitSwitchValue);
    DogLog.log("Elevator/PositionInches", super.positionInches);
    DogLog.log("Elevator/VelocityInchesPerSec", super.velocityInchesPerSec);
  }

  @Override
  public void setState(ElevatorConstants.ElevatorStates state) {
    currentState = state;
    double position =
        MathUtil.clamp(
            SubsystemUtil.elevatorStateToHeight(state),
            ElevatorConstants.elevatorLowerLimit,
            ElevatorConstants.elevatorUpperLimit);
    motionMagicRequest.Position = position;
    leaderMotor.setControl(motionMagicRequest);
  }

  @Override
  public ElevatorConstants.ElevatorStates getState() {
    return super.state;
  }

  @Override
  public void stop() {
    leaderMotor.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            leaderMotor.setNeutralMode(
                enable
                    ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
                    : com.ctre.phoenix6.signals.NeutralModeValue.Coast));
  }

  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void zeroEncoder() {
    leaderMotor.setPosition(0);
    followerMotor.setPosition(0);
  }

  @Override
  public double getPosition() {
    return super.positionInches;
  }
}
