package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import dev.doglog.DogLog;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final DigitalInput hallEffect;
    
    private final MotionMagicVoltage motionMagicRequest;
    
    public ElevatorIOTalonFX() {
        leaderMotor = new TalonFX(ElevatorConstants.leaderMotorID, ElevatorConstants.leaderMotorCANbus);
        followerMotor = new TalonFX(ElevatorConstants.followerMotorID, ElevatorConstants.followerMotorCANbus);
        /*  This tells the motor encoder where 0 inches is*/
        hallEffect = new DigitalInput(ElevatorConstants.hallEffectDioPort);
        
        //followerMotor.setInverted(ElevatorConstants.FOLLOWER_INVERTED);
        followerMotor.setControl(new com.ctre.phoenix6.controls.Follower(ElevatorConstants.leaderMotorID, true));
        
        motionMagicRequest = new MotionMagicVoltage(0);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotionMagic.MotionMagicCruiseVelocity = 
            ElevatorConstants.maxVelocityInchesPerSec / ElevatorConstants.inchesPerCount;
        config.MotionMagic.MotionMagicAcceleration = 
            ElevatorConstants.maxAccelerationInchesPerSecSQ / ElevatorConstants.inchesPerCount;
        
        config.Slot0.kP = ElevatorConstants.kP;
        config.Slot0.kI = ElevatorConstants.kI;
        config.Slot0.kD = ElevatorConstants.kD;
        config.Slot0.kV = ElevatorConstants.kF;
        
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.statorCurrentLimitAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        /* Helps prevent brown outs by limiting current spikes from the battery */
        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.supplyCurrentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        leaderMotor.getConfigurator().apply(config);
        
        enableBrakeMode(true);
    }
    
    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.positionInches = leaderMotor.getPosition().getValueAsDouble() * ElevatorConstants.inchesPerCount;
        inputs.velocityInchesPerSec = leaderMotor.getVelocity().getValueAsDouble() * ElevatorConstants.inchesPerCount;
        inputs.appliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = leaderMotor.getSupplyCurrent().getValueAsDouble();
        inputs.targetPositionInches = motionMagicRequest.Position * ElevatorConstants.inchesPerCount;
        inputs.hallEffectTriggered = !hallEffect.get();
        inputs.tempCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();
        
        /* Determines if the elevator is at a setpoint */
        double positionError = Math.abs(inputs.targetPositionInches - inputs.positionInches);
        double velocityError = Math.abs(inputs.velocityInchesPerSec);
        inputs.atSetpoint = positionError < ElevatorConstants.PositionToleranceInches && 
                           velocityError < ElevatorConstants.velocityToleranceInchesPerSec;
        
        // Determine state
        if (leaderMotor.getControlMode().getValueAsDouble() == 0) {
            /* Elevator is off */
            inputs.state = ElevatorState.DISABLED;
        } else if (leaderMotor.getFault_Hardware().getValue()) {
            /* Elevator is not working lol */
            inputs.state = ElevatorState.ERROR;
        } else if (Math.abs(velocityError) > ElevatorConstants.velocityToleranceInchesPerSec) {
            /* Elevator is moving setpoints */
            inputs.state = ElevatorState.MOVING;
        } else if (inputs.hallEffectTriggered) {
            /* Elevator is being reset back to 0 inches */
            inputs.state = ElevatorState.HOMING;
        } else {
            /* Elevator is currently staying in the same position*/
            inputs.state = ElevatorState.HOLDING;
        }

        /* Log all inputs */
        DogLog.log("Elevator/PositionInches", inputs.positionInches);
        DogLog.log("Elevator/VelocityInchesPerSec", inputs.velocityInchesPerSec);
        DogLog.log("Elevator/TargetPositionInches", inputs.targetPositionInches);
        DogLog.log("Elevator/AppliedVolts", inputs.appliedVolts);
        DogLog.log("Elevator/CurrentAmps", inputs.currentAmps);
        DogLog.log("Elevator/AtSetpoint", inputs.atSetpoint);
        DogLog.log("Elevator/State", inputs.state.toString());
        DogLog.log("Elevator/HallEffect", inputs.hallEffectTriggered);
    }
}