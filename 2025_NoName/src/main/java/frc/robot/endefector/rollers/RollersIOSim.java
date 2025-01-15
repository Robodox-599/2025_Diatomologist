package frc.robot.subsystems.endefector.rollers;
import frc.robot.util.SimLog;
import static frc.robot.subsystems.endefector.rollers.RollersConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.LinearSystemId;


public class RollersIOSim extends RollersIO {
     private final DCMotorSim rollersSim;
     private double desiredVelocity;
     private static final DCMotor ROLLERS_GEARBOX = DCMotor.getKrakenX60Foc(1);

public RollersIOSim(){
  rollersSim = new DCMotorSim(
  LinearSystemId.createDCMotorSystem(ROLLERS_GEARBOX, rollersMOI, gearRatio), ROLLERS_GEARBOX);
}

  @Override
  public void setVoltage(double voltage) {
    rollersSim.setInputVoltage(voltage);
  }

 @Override
 public void setVelocity(double velocity){
  desiredVelocity = velocity;
  rollersSim.setInputVoltage(velocity);
 }

 @Override
 public void stop(){
  setVelocity(0);
 }

 @Override
 public void updateInputs() {
   SimLog.log("Rollers", rollersSim);
   DogLog.log("Rollers/VelocitySetpoint", desiredVelocity);
 }
}

