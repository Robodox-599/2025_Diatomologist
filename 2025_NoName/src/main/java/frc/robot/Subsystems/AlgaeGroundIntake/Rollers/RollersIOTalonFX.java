package frc.robot.subsystems.algaegroundintake.rollers;

import static frc.robot.subsystems.algaegroundintake.rollers.RollersConstants.*;

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

    private final StatusSignal<AngularVelocity> velocityRadsPerSec;
    private final StatusSignal<Voltage> appliedVoltage;
    private double desiredvelocitySetpoint;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> tempCelcius;

    private final VelocityVoltage rollersVelocityVoltage = new VelocityVoltage(0).withSlot(1);

    public RollersIOTalonFX() {
        rollersMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);

        rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
        rollersConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
        rollersConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
        rollersConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

        appliedVoltage = rollersMotor.getSupplyVoltage();
        velocityRadsPerSec = rollersMotor.getVelocity();
        tempCelcius = rollersMotor.getDeviceTemp();
        currentAmps = rollersMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, velocityRadsPerSec, tempCelcius, currentAmps);

        rollersMotor.optimizeBusUtilization();
        rollersMotor.getConfigurator().apply(rollersConfig);
    }

    MotorLog.log("RollersMotor", RollersMotor);

    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(appliedVoltage, velocityRadsPerSec, tempCelcius, currentAmps);

        super.velocitySetpoint = desiredvelocitySetpoint;
        super.appliedVoltage = appliedVoltage.getValueAsDouble();
        super.tempCelcius = tempCelcius.getValueAsDouble();
        super.currentAmps = currentAmps.getValueAsDouble();
        super.velocityRadsPerSec = velocityRadsPerSec.getValueAsDouble();
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
