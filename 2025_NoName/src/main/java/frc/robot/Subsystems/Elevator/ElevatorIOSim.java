package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.ElevatorUtil;
import frc.robot.util.Simlog;

public class ElevatorIOSim extends ElevatorIO {
    private final DCMotorSim elevatorSim;
    private final PIDController positionController;
    private double targetPositionInches = 0.0;
    private boolean brakeMode = true;
    private final PIDController simPidController = 
        new PIDController(ElevatorConstants.simkP, ElevatorConstants.simkI, ElevatorConstants.simkD);


    // private ElevatorState currentState = ElevatorState.HOMING;
    
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
    public void updateInputs() {
        /* Calculate position control output */
        double currentPositionInches = elevatorSim.getAngularPositionRad() * ElevatorConstants.inchesPerCount;
        double voltageOutput = positionController.calculate(currentPositionInches, targetPositionInches);
        voltageOutput = Math.max(-12.0, Math.min(12.0, voltageOutput));
        
        /* Apply voltage if moving or in brake mode */
        if (brakeMode || Math.abs(elevatorSim.getAngularAccelerationRadPerSecSq() * ElevatorConstants.inchesPerCount) > ElevatorConstants.velocityToleranceInchesPerSec) {
            elevatorSim.setInputVoltage(voltageOutput);
        }
        
        elevatorSim.update(0.02);
        
        // Update inputs structure
        super.positionInches = elevatorSim.getAngularPositionRad() * ElevatorConstants.inchesPerCount;
        super.velocityInchesPerSec = elevatorSim.getAngularAccelerationRadPerSecSq() * ElevatorConstants.inchesPerCount;
        super.appliedVolts = elevatorSim.getCurrentDrawAmps() * ElevatorConstants.nominal_voltage;
        super.currentAmps = elevatorSim.getCurrentDrawAmps();
        super.targetPositionInches = targetPositionInches;
        super.tempCelsius = 25.0;
        
        /* Checks if elevator is at setpoint */
        super.atSetpoint = positionController.atSetpoint();
        
        // Update state
        Simlog.log("elevatorMotors", elevatorSim);

        DogLog.log("Elevator/PositionInches", super.positionInches);
        DogLog.log("Elevator/VelocityInchesPerSec", super.velocityInchesPerSec);
        DogLog.log("Elevator/AppliedVolts", super.appliedVolts);
        DogLog.log("Elevator/TargetPositionInches", super.targetPositionInches);
        DogLog.log("Elevator/AtSetpoint", super.atSetpoint);
        DogLog.log("Elevator/State", super.state.toString());
    }
    
    @Override
    public void setState(ElevatorConstants.ElevatorStates state) {
        double position = MathUtil.clamp(ElevatorUtil.stateToHeight(state), ElevatorConstants.elevatorLowerLimit, ElevatorConstants.elevatorUpperLimit);
        
        elevatorSim.setInputVoltage(simPidController.calculate(position));
    }

    @Override
    public void stop() {
        elevatorSim.setInputVoltage(0);
    }

    @Override
    public void enableBrakeMode(boolean enable) {
        brakeMode = enable;
    }

    @Override
    public void setVoltage(double voltage){
        elevatorSim.setInputVoltage(voltage);
    }

    @Override
    public void zeroEncoder(){
        
    }

    @Override
    public double getPosition(){
        return elevatorSim.getAngularPositionRad() * ElevatorConstants.inchesPerCount;
    }
}