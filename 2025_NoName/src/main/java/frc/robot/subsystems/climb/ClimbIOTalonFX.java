package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.ClimbUtil;
import frc.robot.util.MotorLog;
import frc.robot.util.PhoenixUtil;

public class ClimbIOTalonFX extends ClimbIO {

  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  private final DigitalInput limitSwitch;
  private ClimbConstants.ClimbStates currentState = ClimbConstants.ClimbStates.CLIMBREADY;

  private final MotionMagicVoltage motionMagicRequest;
  private int motionSlot;

  public ClimbIOTalonFX() {
    leaderMotor = new TalonFX(ClimbConstants.leaderMotorID, ClimbConstants.leaderMotorCANbus);
    followerMotor = new TalonFX(ClimbConstants.followerMotorID, ClimbConstants.followerMotorCANbus);
    /*  This tells the motor encoder where 0 inches is*/
    limitSwitch = new DigitalInput(ClimbConstants.limitSwitchDioPort);

    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), true));

    motionMagicRequest = new MotionMagicVoltage(0);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotionMagic.MotionMagicCruiseVelocity =
        ClimbConstants.maxVelocityInchesPerSec / ClimbConstants.inchesPerRev;
    config.MotionMagic.MotionMagicAcceleration =
        ClimbConstants.maxAccelerationInchesPerSecSQ / ClimbConstants.inchesPerRev;

    config.Slot0.kP = ClimbConstants.kP;
    config.Slot0.kI = ClimbConstants.kI;
    config.Slot0.kD = ClimbConstants.kD;
    config.Slot0.kV = ClimbConstants.kV;
    config.Slot0.kS = ClimbConstants.kS;

    config.CurrentLimits.StatorCurrentLimit = ClimbConstants.statorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    /* Helps prevent brown outs by limiting current spikes from the battery */
    config.CurrentLimits.SupplyCurrentLimit = ClimbConstants.supplyCurrentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // enableBrakeMode(true);
    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.setPosition(0.0, 0.25));
    leaderMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs() {
    super.positionInches =
        leaderMotor.getPosition().getValueAsDouble() * ClimbConstants.inchesPerRev;
    super.velocityInchesPerSec =
        leaderMotor.getVelocity().getValueAsDouble() * ClimbConstants.inchesPerRev;
    super.appliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();
    super.currentAmps = leaderMotor.getSupplyCurrent().getValueAsDouble();
    // super.targetPositionInches = motionMagicRequest.Position * ClimbConstants.inchesPerRev;
    super.tempCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();
    super.state = currentState;
    /* Determines if the elevator is at a setpoint */
    double positionError = Math.abs(super.targetPositionInches - super.positionInches);
    double velocityError = Math.abs(super.velocityInchesPerSec);
    super.atSetpoint =
        positionError < ClimbConstants.PositionToleranceInches
            && velocityError < ClimbConstants.velocityToleranceInchesPerSec;

    super.limitSwitchValue = limitSwitch.get();
    /*Log basic motor inputs */
    MotorLog.log("leaderMotor", leaderMotor);
    MotorLog.log("followerMotor", followerMotor);

    /* Log all super */
    DogLog.log("Climb/TargetPositionInches", super.targetPositionInches);
    DogLog.log("Climb/AtSetpoint", super.atSetpoint);
    DogLog.log("Climb/State", super.state.toString());
    DogLog.log("Climb/LimitSwitchValue", super.limitSwitchValue);
    DogLog.log("Climb/PositionInches", super.positionInches);
    DogLog.log("Climb/VelocityInchesPerSec", super.velocityInchesPerSec);
  }

  @Override
  public void setState(ClimbConstants.ClimbStates state) {
    currentState = state;
    double position =
        MathUtil.clamp(
            ClimbUtil.stateToHeight(state),
            ClimbConstants.climbLowerLimit,
            ClimbConstants.climbUpperLimit);

    ClimbUtil.stateToHeight(state);

    if (position > getPositionInches()) {
      motionSlot = ClimbConstants.movingUpSlot;
    } else {
      motionSlot = ClimbConstants.movingDownSlot;
    }
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
  }

  @Override
  public double getPositionInches() {
    return leaderMotor.getPosition().getValueAsDouble();
  }
}
