package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import frc.robot.util.MotorLog;

public class RollersIOReal implements RollersIO {
  private TalonFX rollerMotor;
  TalonFXConfiguration rollerConfig;
  private double desiredVelo;

  public RollersIOReal() {
    rollerMotor = new TalonFX(RollerConstants.motorID, RollerConstants.motorCANBus);
    rollerConfig = new TalonFXConfiguration();
    rollerConfig.Slot0.kP = RollerConstants.realkP;
    rollerConfig.Slot0.kI = RollerConstants.realkI;
    rollerConfig.Slot0.kD = RollerConstants.realkD;
    rollerConfig.Slot0.kS = RollerConstants.realkS;
    rollerConfig.Slot0.kV = RollerConstants.realkV;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = RollerConstants.EnableSupplyurrentLimit;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.PeakSupplyCurrentLimit;
    rollerConfig.CurrentLimits.SupplyCurrentLowerLimit = RollerConstants.LowerSupplyCurrentLimit;
    rollerConfig.CurrentLimits.SupplyCurrentLowerTime = RollerConstants.LowerSupplyCurrentDuration;

    // optimize comms between Talons and CAN bus
    rollerMotor.getConfigurator().apply(rollerConfig);
    rollerMotor.optimizeBusUtilization();
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs() {
    MotorLog.log(
        "Rollers",
        rollerMotor); // logs all motor data for us under the same key as the rest of the data :D
    DogLog.log(
        "Rollers/VelocitySetpoint",
        desiredVelo); // log anything else, like desired velocity, under the same key and ur done
  }

  /* sets motor voltage if needed, will prolly not be used */
  @Override
  public void setVoltage(double voltage) {
    rollerMotor.setVoltage(voltage);
  }

  /* sets motor velo to 0 to stop the motor */
  @Override
  public void stop() {
    runVelocity(0);
  }

  /** sets velocity of motor */
  @Override
  public void runVelocity(double speed) {
    desiredVelo = speed;
    rollerMotor.set(speed);
  }
}
