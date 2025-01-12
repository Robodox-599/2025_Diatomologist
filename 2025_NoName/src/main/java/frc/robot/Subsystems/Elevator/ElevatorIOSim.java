package frc.robot.Subsystems.Elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
//import frc.robot.util.SimLog;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotorSim elevatorSim;
    private final PIDController positionController;
    private double targetPositionInches = 0.0;
    private boolean brakeMode = true;
    private ElevatorState currentState = ElevatorState.DISABLED;
    
    private static final DCMotor ELEVATOR_GEARBOX = DCMotor.getKrakenX60Foc(2);
    
    public ElevatorIOSim() {
        elevatorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ELEVATOR_GEARBOX,
                ElevatorConstants.elevatorMOI,
                ElevatorConstants.gearRatio),
            ELEVATOR_GEARBOX);
        
        positionController = new PIDController(
            ElevatorConstants.simkP,
            ElevatorConstants.simkI,
            ElevatorConstants.simkD
        );
        positionController.setTolerance(
            ElevatorConstants.PositionToleranceInches,
            ElevatorConstants.velocityToleranceInchesPerSec
        );
    }
    
    @Override
    public void updateInputs(ElevatorInputs inputs) {
        if (currentState != ElevatorState.DISABLED) {
            /* Calculate position control output */
            double currentPositionInches = getPositionInches();
            double voltageOutput = positionController.calculate(currentPositionInches, targetPositionInches);
            voltageOutput = Math.max(-12.0, Math.min(12.0, voltageOutput));
            
            /* Apply voltage if moving or in brake mode */
            if (brakeMode || Math.abs(getVelocityInchesPerSec()) > ElevatorConstants.velocityToleranceInchesPerSec) {
                elevatorSim.setInputVoltage(voltageOutput);
            }
        }
        
        elevatorSim.update(0.02);
        
        // Update inputs structure
        inputs.positionInches = getPositionInches();
        inputs.velocityInchesPerSec = getVelocityInchesPerSec();
        inputs.appliedVolts = elevatorSim.getCurrentDrawAmps() * ElevatorConstants.nominal_voltage;
        inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
        inputs.targetPositionInches = targetPositionInches;
        inputs.hallEffectTriggered = getPositionInches() <= 1.0;
        inputs.tempCelsius = 25.0;
        
        /* Checks if elevator is at setpoint */
        inputs.atSetpoint = positionController.atSetpoint();
        
        // Update state
        updateState(inputs);
        
        // Log simulation data
        //SimLog.log("Elevator", elevatorSim);
        DogLog.log("Elevator/PositionInches", inputs.positionInches);
        DogLog.log("Elevator/VelocityInchesPerSec", inputs.velocityInchesPerSec);
        DogLog.log("Elevator/TargetPositionInches", inputs.targetPositionInches);
        DogLog.log("Elevator/AppliedVolts", inputs.appliedVolts);
        DogLog.log("Elevator/CurrentAmps", inputs.currentAmps);
        DogLog.log("Elevator/AtSetpoint", inputs.atSetpoint);
        DogLog.log("Elevator/State", inputs.state.toString());
        DogLog.log("Elevator/HallEffect", inputs.hallEffectTriggered);
    }
    
    private void updateState(ElevatorInputs inputs) {
        if (currentState == ElevatorState.DISABLED) {
            inputs.state = ElevatorState.DISABLED;
        } else if (inputs.hallEffectTriggered) {
            inputs.state = ElevatorState.HOMING;
        } else if (Math.abs(inputs.velocityInchesPerSec) > ElevatorConstants.velocityToleranceInchesPerSec) {
            inputs.state = ElevatorState.MOVING;
        } else {
            inputs.state = ElevatorState.HOLDING;
        }
    }
    
    private double getPositionInches() {
        return elevatorSim.getAngularPositionRad() * ElevatorConstants.drumRadiusMeters * Units.metersToInches(1);
    }
    
    private double getVelocityInchesPerSec() {
        return elevatorSim.getAngularVelocityRadPerSec() * ElevatorConstants.drumRadiusMeters * Units.metersToInches(1);
    }
    
    @Override
    public void setHeight(double heightInches) {
        targetPositionInches = heightInches;
        currentState = ElevatorState.MOVING;
    }
    
    @Override
    public void MotionMagicVoltage(double volts) {
        elevatorSim.setInputVoltage(volts);
    }
    
    @Override
    public void setEncoder(double positionInches) {
        elevatorSim.setState(
            positionInches / (ElevatorConstants.drumRadiusMeters * Units.metersToInches(1)),
            elevatorSim.getAngularVelocityRadPerSec());
    }
    
    @Override
    public void stop() {
        elevatorSim.setInputVoltage(0.0);
        currentState = ElevatorState.DISABLED;
    }
    
    @Override
    public void enableBrakeMode(boolean enable) {
        brakeMode = enable;
    }
    
    @Override
    public void zeroEncoder() {
        setEncoder(0.0);
    }
    
    @Override
    public void configurePID(double kP, double kI, double kD) {
        positionController.setP(kP);
        positionController.setI(kI);
        positionController.setD(kD);
    }
}