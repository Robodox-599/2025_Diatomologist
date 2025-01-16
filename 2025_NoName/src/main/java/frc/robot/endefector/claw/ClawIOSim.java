package frc.robot.endefector.claw;
import static frc.robot.endefector.claw.ClawConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.SimLog;

public class ClawIOSim extends ClawIO{
    private static final DCMotor CLAW_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private final DCMotorSim clawSim;
 
    private double passedInPositon;

    private final PIDController clawPID = 
    new PIDController(simkP, simkI, simkD);

    public ClawIOSim() {
        clawSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            CLAW_GEARBOX, clawMOI, gearRatio), CLAW_GEARBOX);
    }
    
    @Override
    public void updateInputs() {
        SimLog.log("ClawVelocity", clawSim);
        SimLog.log("ClawVoltage", clawSim);
        SimLog.log("ClawAmps", clawSim);
        SimLog.log("ClawTemp", clawSim);
        SimLog.log("ClawPosition", clawSim);

        DogLog.log("Claw/TargetPosition", passedInPositon);
    }

    
    @Override
    public void goToPose(double pose) {
        clawSim.setInputVoltage(clawPID.calculate(pose));
    }

    @Override
    public void setVoltage(double voltage){
        clawSim.setInputVoltage(voltage);
    }

    @Override
    public double getPose(){
        return clawSim.getAngularPositionRad();
    }

    @Override
    public void stop(){
        clawSim.setAngularVelocity(0);
    }
}
