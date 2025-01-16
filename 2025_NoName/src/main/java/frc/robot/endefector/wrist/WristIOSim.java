package frc.robot.endefector.wrist;
import static frc.robot.endefector.wrist.WristConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.SimLog;

public class WristIOSim extends WristIO {
    
    private static final DCMotor WRIST_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private final DCMotorSim wristSim;
 

    private double passedInPositon;
    private double currentPosition;

    private final PIDController wristPID = 
    new PIDController(simkP, simkI, simkD);

    public WristIOSim() {
        wristSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            WRIST_GEARBOX, wristMOI, gearRatio), WRIST_GEARBOX);

    }

    @Override
    public void updateInputs() {
        SimLog.log("WristVelocity", wristSim);
        SimLog.log("WristVoltage", wristSim);
        SimLog.log("WristAmps", wristSim);
        SimLog.log("WristTemp", wristSim);
        SimLog.log("WristPosition", wristSim);

        DogLog.log("Wrist/TargetPosition", passedInPositon);
        DogLog.log("Wrist/CurrentPosition", currentPosition);
    }

    @Override
    public void goToPose(double position) {
        wristSim.setInputVoltage(wristPID.calculate(position));
    }

    @Override
    public void setVoltage(double voltage){
        wristSim.setInputVoltage(voltage);
    }

    @Override
    public double getPose(){
        return wristSim.getAngularPositionRad();
    }

    @Override
    public void stop(){
        wristSim.setAngularVelocity(0);
    }
}
