package frc.robot.endefector.wrist;
import frc.robot.util.MotorLog;
import static frc.robot.endefector.wrist.WristConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import dev.doglog.DogLog;

// import edu.wpi.first.math.MathUtil;

public class WristIOTalonFX extends WristIO{
    
  private final TalonFX wristMotor;
  TalonFXConfiguration wristConfig;
  private final MotionMagicVoltage m_request;

  private double passedInPosition;
  private double currentPosition;
  private int wristSlot;

    public WristIOTalonFX() {
    
        wristMotor = new TalonFX(wristMotorID, wristMotorCANBus);
        wristConfig = new TalonFXConfiguration();
        m_request = new MotionMagicVoltage(0);

        var motionMagicConfigs = wristConfig.MotionMagic;
        //I don't really know what values to put here :(
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
        motionMagicConfigs.MotionMagicAcceleration = 0.0;

        wristConfig.Slot0.kP = realExtendkP;
        wristConfig.Slot0.kI = realExtendkI;
        wristConfig.Slot0.kD = realExtendkD;
        wristConfig.Slot0.kV = realExtendkV;

        wristConfig.Slot1.kP = realRetractkP;
        wristConfig.Slot1.kI = realRetractkI;
        wristConfig.Slot1.kD = realRetractkD;
        wristConfig.Slot1.kV = realRetractkV;
    
        wristConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
        wristConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
        wristConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
        wristConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;
    
    wristMotor.optimizeBusUtilization();

    wristMotor.getConfigurator().apply(wristConfig);
    }

    @Override
    public void updateInputs() {
      MotorLog.log("Wrist", wristMotor);
      DogLog.log("Wrist/TargetPosition", passedInPosition);
      DogLog.log("Wrist/CurrentPosition", currentPosition);
  }

    @Override
    public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

    @Override
    public void goToPose(double position){
      passedInPosition = position;
      
      if (passedInPosition > currentPosition) {
        wristSlot = 0;
      } else {
        wristSlot = 1;
      }
      
      m_request.withSlot(wristSlot);
      wristMotor.setControl(m_request);
   }

    @Override
    public double getPose(){
      return wristMotor.getPosition().getValueAsDouble();
    }

  @Override
  public void stop() {
      wristMotor.stopMotor();
  }

  @Override
    public void setBrake(boolean brake) {
    wristMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
