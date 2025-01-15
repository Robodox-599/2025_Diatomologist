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
    //private boolean brakeMode = true;
    private final PIDController simPidController = 
        new PIDController(ElevatorConstants.simkP, ElevatorConstants.simkI, ElevatorConstants.simkD);

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
        elevatorSim.update(0.02);
        
        // Update inputs structure
        super.positionInches = elevatorSim.getAngularPositionRad() * ElevatorConstants.inchesPerRev;
        super.velocityInchesPerSec = elevatorSim.getAngularAccelerationRadPerSecSq() * ElevatorConstants.inchesPerRev;
        super.appliedVolts = elevatorSim.getCurrentDrawAmps() * ElevatorConstants.nominal_voltage;
        super.currentAmps = elevatorSim.getCurrentDrawAmps();
        super.targetPositionInches = targetPositionInches;
        super.tempCelsius = 25.0; // setting 
        
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

    // @Override
    // public void enableBrakeMode(boolean enable) {
    //     brakeMode = enable;
    // }

    @Override
    public void setVoltage(double voltage){
        elevatorSim.setInputVoltage(voltage);
    }

    @Override
    public void zeroEncoder(){
        elevatorSim.setAngle(0);
    }

    @Override
    public double getPosition(){
        return elevatorSim.getAngularPositionRad() * ElevatorConstants.inchesPerRev;
    }
}