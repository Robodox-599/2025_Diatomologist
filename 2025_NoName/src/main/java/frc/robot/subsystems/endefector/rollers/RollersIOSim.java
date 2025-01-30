package frc.robot.subsystems.endefector.rollers;
import frc.robot.util.SimLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static frc.robot.subsystems.endefector.rollers.RollersConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.LinearSystemId;
//editing file

public class RollersIOSim extends RollersIO {
     private final DCMotorSim rollersSim;
     private double desiredVelocity;
     private PIDController rollerController =
      new PIDController(simkP, simkI, simkD);
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
  rollersSim.setInputVoltage(
        rollerController.calculate(rollersSim.getAngularVelocityRPM() / 60.0, velocity));
  }

 @Override
 public void stop(){
  setVelocity(0);
 }

 @Override
 public void updateInputs() {
   super.appliedVolts = rollersSim.getInputVoltage();
   super.currentAmps = rollersSim.getCurrentDrawAmps();
   super.velocity = rollersSim.getAngularVelocityRPM() / 60.0;
   super.desiredVelocity = desiredVelocity;
   super.tempCelsius = 25.0;

   SimLog.log("RollersSimMotor", rollersSim);
   DogLog.log("Rollers/VelocitySetpoint", desiredVelocity);
 }
}