package frc.robot.endefector.claw;

import frc.robot.util.MotorLog;
import static frc.robot.endefector.claw.ClawConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

import dev.doglog.DogLog;

public class ClawIOTalonFX extends ClawIO {
  private final TalonFX clawMotor;
  TalonFXConfiguration clawConfig;
    private final PositionVoltage clawPositionVoltage =
     new PositionVoltage(0).withSlot(0);

  private double passedInPosition;

  public ClawIOTalonFX() {
    
    clawMotor = new TalonFX(clawMotorID, clawMotorCANBus);
    clawConfig = new TalonFXConfiguration();

    clawConfig.Slot0.kP = realkP;
    clawConfig.Slot0.kI = realkI;
    clawConfig.Slot0.kD = realkD;
    clawConfig.Slot0.kV = realkV;

    clawConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    clawConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    clawConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    clawConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    clawMotor.optimizeBusUtilization();

    clawMotor.getConfigurator().apply(clawConfig);
  }

  @Override
  public void updateInputs(){
    MotorLog.log("Claw", clawMotor);
    DogLog.log("Claw/TargetPosition", passedInPosition);
  }

  @Override
  public void setVoltage(double voltage) {
      clawMotor.setVoltage(voltage);
  }

  @Override
  public void stop() {
      clawMotor.setVoltage(0);
  }

    @Override
    public void setBrake(boolean brake) {
    clawMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public double getPose(){
    return clawMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void goToPose(double position){
    passedInPosition = position;
    clawMotor.setControl(clawPositionVoltage.withPosition(Units.radiansToRotations(position)));
 }
}
