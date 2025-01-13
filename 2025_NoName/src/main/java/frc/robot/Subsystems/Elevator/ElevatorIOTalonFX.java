package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.Motorlog;
import frc.robot.util.ElevatorUtil;
import dev.doglog.DogLog;


public class ElevatorIOTalonFX extends ElevatorIO {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final DigitalInput limitSwitch;
    private ElevatorConstants.ElevatorStates currentState;
    private final MotionMagicVoltage motionMagicRequest;
    private int motionSlot;
    public ElevatorIOTalonFX() {
        leaderMotor = new TalonFX(ElevatorConstants.leaderMotorID, ElevatorConstants.leaderMotorCANbus);
        followerMotor = new TalonFX(ElevatorConstants.followerMotorID, ElevatorConstants.followerMotorCANbus);
        /*  This tells the motor encoder where 0 inches is*/
        limitSwitch = new DigitalInput(ElevatorConstants.limitSwitchDioPort);
        
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
    public void updateInputs() {
        super.positionInches = leaderMotor.getPosition().getValueAsDouble() * ElevatorConstants.inchesPerCount;
        super.velocityInchesPerSec = leaderMotor.getVelocity().getValueAsDouble() * ElevatorConstants.inchesPerCount;
        super.appliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();
        super.currentAmps = leaderMotor.getSupplyCurrent().getValueAsDouble();
        super.targetPositionInches = motionMagicRequest.Position * ElevatorConstants.inchesPerCount;
        super.tempCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();
        super.state = currentState;
        /* Determines if the elevator is at a setpoint */
        double positionError = Math.abs(super.targetPositionInches - super.positionInches);
        double velocityError = Math.abs(super.velocityInchesPerSec);
        super.atSetpoint = positionError < ElevatorConstants.PositionToleranceInches && 
                           velocityError < ElevatorConstants.velocityToleranceInchesPerSec;
                           
        super.limitSwitchValue = limitSwitch.get();
        /* Log all super */
        Motorlog.log("leaderMotor", leaderMotor);
        Motorlog.log("followerMotor", followerMotor);
    
        DogLog.log("Elevator/TargetPositionInches", super.targetPositionInches);
        DogLog.log("Elevator/AtSetpoint", super.atSetpoint);
        DogLog.log("Elevator/State", super.state.toString());
        DogLog.log("Elevator/LimitSwitchValue", super.limitSwitchValue);
    }

    @Override
    public void setState(ElevatorConstants.ElevatorStates state) {
        currentState = state;
        double position = MathUtil.clamp(ElevatorUtil.stateToHeight(state), ElevatorConstants.elevatorLowerLimit, ElevatorConstants.elevatorUpperLimit);
        
        if (position > getPosition()){
            motionSlot = ElevatorConstants.movingUpSlot;
        } else {
            motionSlot = ElevatorConstants.movingDownSlot;
        }

        motionMagicRequest.withSlot(motionSlot);
        motionMagicRequest.Position = position;
        leaderMotor.setControl(motionMagicRequest);
    }

    @Override
    public void stop() {
        leaderMotor.stopMotor();
    }

    @Override
    public void enableBrakeMode(boolean enable) {
        leaderMotor.setNeutralMode(enable ? 
            com.ctre.phoenix6.signals.NeutralModeValue.Brake : 
            com.ctre.phoenix6.signals.NeutralModeValue.Coast);
        followerMotor.setNeutralMode(enable ? 
            com.ctre.phoenix6.signals.NeutralModeValue.Brake : 
            com.ctre.phoenix6.signals.NeutralModeValue.Coast);
    }

    @Override
    public void setVoltage(double voltage){
        leaderMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void zeroEncoder(){
        leaderMotor.setPosition(0);
    }

    @Override
    public double getPosition(){
        return leaderMotor.getPosition().getValueAsDouble();
    }
}