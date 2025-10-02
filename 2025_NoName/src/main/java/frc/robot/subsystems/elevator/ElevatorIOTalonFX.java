package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class ElevatorIOTalonFX extends ElevatorIO {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  // private final DigitalInput limitSwitch1;
  // private final DigitalInput limitSwitch2;
  private final MotionMagicVoltage motionMagicRequest;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  // private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temperature;

  public ElevatorIOTalonFX() {
    leaderMotor = new TalonFX(ElevatorConstants.leaderMotorID, ElevatorConstants.leaderMotorCANbus);
    followerMotor =
        new TalonFX(ElevatorConstants.followerMotorID, ElevatorConstants.followerMotorCANbus);

    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), true));

    motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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

    position = leaderMotor.getPosition();
    velocity = leaderMotor.getVelocity();
    // acceleration = leaderMotor.getAcceleration();
    appliedVolts = leaderMotor.getMotorVoltage();
    current = leaderMotor.getStatorCurrent();
    temperature = leaderMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, velocity, temperature, position, current, appliedVolts);

    leaderMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();

    zeroEncoder();
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(velocity, temperature, position, current, appliedVolts);
    super.positionInches = position.getValueAsDouble() * ElevatorConstants.inchesPerRev;
    super.velocityInchesPerSec = velocity.getValueAsDouble() * ElevatorConstants.inchesPerRev;
    super.appliedVolts = appliedVolts.getValueAsDouble();
    // super.acceleration = acceleration.getValueAsDouble() * ElevatorConstants.inchesPerRev;
    super.currentAmps = current.getValueAsDouble();
    super.targetPositionInches = motionMagicRequest.Position * ElevatorConstants.inchesPerRev;

    /* Determines if the elevator is at a setpoint */
    super.atSetpoint =
        Math.abs(super.targetPositionInches - super.positionInches)
            < ElevatorConstants.positionToleranceInches;

    DogLog.log("Elevator/StatorCurrentAmps", super.currentAmps);
    DogLog.log("Elevator/AppliedVoltage", super.appliedVolts);
    DogLog.log("Elevator/TempCelcius", super.tempCelsius);
    DogLog.log("Elevator/PositionInches", super.positionInches);
    DogLog.log("Elevator/TargetPositionInches", super.targetPositionInches);
    DogLog.log("Elevator/ElevatorAtSetpoint", super.atSetpoint);
    DogLog.log("Elevator/VelocityInchesPerSec", super.velocityInchesPerSec);
  }

  @Override
  public void setHeight(ElevatorConstants.ElevatorStates state) {
    double position =
        MathUtil.clamp(
            SubsystemUtil.elevatorStateToHeightTicks(state),
            super.elevatorSoftLowerLimit,
            super.elevatorSoftUpperLimit);
    position =
        MathUtil.clamp(
            position,
            ElevatorConstants.elevatorHardLowerLimit,
            ElevatorConstants.elevatorHardUpperLimit);
    motionMagicRequest.Position = position;
    leaderMotor.setControl(motionMagicRequest);
  }

  @Override
  public void stop() {
    leaderMotor.stopMotor();
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
}
