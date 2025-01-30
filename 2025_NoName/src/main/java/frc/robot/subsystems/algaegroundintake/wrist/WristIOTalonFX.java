package frc.robot.subsystems.algaegroundintake.wrist;

// import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import frc.robot.util.MotorLog;
import frc.robot.util.PhoenixUtil;

public class WristIOTalonFX extends WristIO {
    
    private final TalonFX intakeWristMotor = new TalonFX(WristConstants.wristMotorID, WristConstants.wristMotorCANBus);

    private double setPoint = 0.0;
    private double motorEncoder;
    private int m_WristSlot = 0;


    public WristIOTalonFX() {
        var config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var MotionMagicConfigs = config.MotionMagic;

        MotionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
        MotionMagicConfigs.MotionMagicAcceleration = 0.0;

        config.Slot0.kP = WristConstants.wristExtendKP;
        config.Slot0.kI = WristConstants.wristExtendKI;
        config.Slot0.kD = WristConstants.wristExtendKD;
        config.Slot0.kS = WristConstants.wristExtendKS;

        config.Slot1.kP = WristConstants.wristRetractKP;
        config.Slot1.kI = WristConstants.wristRetractKI;
        config.Slot1.kD = WristConstants.wristRetractKD;

        var mmConfig = config.MotionMagic;
        mmConfig.MotionMagicCruiseVelocity = WristConstants.maxWristVelocity;
        mmConfig.MotionMagicAcceleration = WristConstants.maxWristAccel;

        intakeWristMotor.getConfigurator().apply(config);
        intakeWristMotor.setPosition(0);
        setBrake(true);

        motorEncoder = intakeWristMotor.getPosition().getValueAsDouble();
        // intakeWristMotor.optimizeBusUtilization();
         PhoenixUtil.tryUntilOk(5, ()-> intakeWristMotor.getConfigurator().apply(config));
    }

    @Override
    public void updateInputs() {
      super.appliedVolts = intakeWristMotor.getMotorVoltage().getValueAsDouble();
      super.currentAmps = intakeWristMotor.getSupplyCurrent().getValueAsDouble();
      super.velocity = intakeWristMotor.getVelocity().getValueAsDouble();
      super.tempCelsius = intakeWristMotor.getDeviceTemp().getValueAsDouble();
      super.position = intakeWristMotor.getPosition().getValueAsDouble();
      super.targetPosition = targetPosition;
      super.currentPosition = currentPosition;

      MotorLog.log("Wrist", intakeWristMotor);
    
      DogLog.log("Wrist/TargetPosition", targetPosition);
      DogLog.log("Wrist/CurrentPosition", currentPosition);
      DogLog.log("Wrist/Position", intakeWristMotor.getPosition().getValueAsDouble());
    }

    @Override 
    public void setVoltage(double motorVolts) {
        intakeWristMotor.setVoltage(motorVolts);
    }

    @Override
    public double getAngle() {
        return (Units.radiansToRotations(motorEncoder));
    }

    public void desiredWristSetPos(double passedInPosition) {
        m_WristSlot = 
            passedInPosition == WristConstants.kWristExtendVal
            ? WristConstants.wristExtendSlot
            : WristConstants.wristRetractSlot;
        setPoint = passedInPosition;
        MotionMagicVoltage m_request = 
            new MotionMagicVoltage(setPoint).withSlot(m_WristSlot).withFeedForward(WristConstants.kWristFeedForward);
            intakeWristMotor.setControl(m_request);
    }

    @Override 
    public void goToSetpoint(double setPoint) {
        desiredWristSetPos(setPoint);

    }

    @Override
    public void holdToSetpoint(double setPoint) {
        goToSetpoint(setPoint);
    }

    @Override
    public void setBrake(boolean brake) {
        if(brake) {
            intakeWristMotor.setNeutralMode(NeutralModeValue.Brake);
        }
    }

    @Override 
    public boolean atSetpoint() {
        return Math.abs(getAngle() - setPoint) < WristConstants.intakeWristPositionTolerance;
    }
}
