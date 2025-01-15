package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;


public class RollersIOTalonFX extends RollersIO {
     private TalonFX rollersMotor;
    TalonFXConfiguration rollersConfig;

    private final StatusSignal<Voltage> velocityRadsPerSec;
    private final StatusSignal<AngularVelocity> appliedVoltage;
    private double desiredvelocitySetpoint;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Tempurature> tempCelcius;

    private final VelocityVoltage rollersVelocityVoltage = new VelocityVoltage(0).withSlot(1);

    public RollersIOTalonFX() {
        rollerMotor = new TalonFX(rollerMotorID, rollersMotorCANBus);

        RollersConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
        RollersConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
        RollersConfig.CurrentLimits.SupplyCurrentThreshold = PeakCurrentLimit;
        RollersConfig.CurrentLimits.SupplyTimeThreshold = PeakCurrentDuration;

        appliedVoltage = rollersMotor.getSupplyVoltage();
        velocityRadsPerSec = rollersMotor.getVelocity();
        tempCelcius = rollersMotor.getDeviceTemp();
        currentAmps = rollersMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, velocityRadsPerSec, tempCelcius, currentAmps);

        rollersMotor.optimizeBusUtilization();
        rollersMotor.getConfigurator().apply(RollersConfig);
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
        BaseStatusSignal.refreshAll(appliedVoltage, velocityRadsPerSec, tempCelcius, currentAmps);

        inputs.velocitySetpoint = desiredvelocitySetpoint;
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.tempCelcius = tempCelcius.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.velocityRadsPerSec = velocityRadsPerSec.getValueAsDouble();
    }
    @Override
    public void setVoltage(double voltage){
      rollersMotor.setVoltage(voltage);
    }
    public void setBrake(boolean brake) {
        if (brake) {
            rollersMotor.setNeutralMode(NeutralModeValue.Brake);
         } else {
            rollersMotor.setNeutralMode(NeutralModeValue.Coast);
        }
  }

  @Override
  public void setVelocity(double velocity) {
    rollersMotor.setControl(rollersVelocityVoltage.withVelocity(Units.radiansPerSecondToRotationsPerMinute(velocity)/60));
  }

  @Override
  public void stop() {
    rollersMotor.stopMotor();
  }
}   
