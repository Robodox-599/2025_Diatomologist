package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class ClimbIOTalonFX extends ClimbIO {

  private final TalonFX leaderMotor;
  private final DigitalInput limitSwitch;
  private ClimbConstants.ClimbStates currentState = ClimbConstants.ClimbStates.CLIMBREADY;

  private final BangBangController bangBangController;

  public ClimbIOTalonFX() {
    leaderMotor = new TalonFX(ClimbConstants.leaderMotorID, ClimbConstants.leaderMotorCANbus);
    bangBangController = new BangBangController();

    /*  This tells the motor encoder where 0 inches is*/
    limitSwitch = new DigitalInput(ClimbConstants.limitSwitchDioPort);

    TalonFXConfiguration config = new TalonFXConfiguration();

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
        positionError < ClimbConstants.positionToleranceInches
            && velocityError < ClimbConstants.velocityToleranceInchesPerSec;

    super.limitSwitchValue = limitSwitch.get();
    // Leader motor
    DogLog.log("ClimbLeader/StatorCurrentAmps", super.currentAmps);
    DogLog.log("ClimbLeader/AppliedVoltage", super.appliedVolts);
    DogLog.log("ClimbLeader/TempCelcius", super.tempCelsius);

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
    double targetPositionInches =
        MathUtil.clamp(
            SubsystemUtil.climbStateToHeight(state),
            ClimbConstants.climbLowerLimit,
            ClimbConstants.climbUpperLimit);

    leaderMotor.set(bangBangController.calculate(targetPositionInches));
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
