package frc.robot.subsystems.algaegroundintake.intakeRollers;

import static frc.robot.subsystems.algaegroundintake.intakeRollers.IntakeRollersConstants.*;
// import static frc.robot.subsystems.algaegroundintake.wrist.WristIOSim.*;
import static frc.robot.subsystems.algaegroundintake.intakewrist.IntakeWristConstants.simkD;
import static frc.robot.subsystems.algaegroundintake.intakewrist.IntakeWristConstants.simkI;
import static frc.robot.subsystems.algaegroundintake.intakewrist.IntakeWristConstants.simkP;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.SimLog;


public class IntakeRollersIOSim extends IntakeRollersIO {
    private final DCMotorSim intakeRollersSim;
    private double desiredVelocity;
    private PIDController rollerController =
    new PIDController(simkP, simkI, simkD);
    private static final DCMotor ROLLERS_GEARBOX = DCMotor.getKrakenX60Foc(1);

    public IntakeRollersIOSim(){
      intakeRollersSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(ROLLERS_GEARBOX, rollersMOI, gearRatio), ROLLERS_GEARBOX);
    }

      @Override
      public void setVoltage(double voltage) {
        intakeRollersSim.setInputVoltage(voltage);
      }

     @Override
     public void setVelocity(double velocity){
      desiredVelocity = velocity;
      intakeRollersSim.setInputVoltage(
            rollerController.calculate(intakeRollersSim.getAngularVelocityRPM() / 60.0, velocity));
      }

     @Override
     public void stop(){
      setVelocity(0);
     }

     @Override
     public void updateInputs() {
       super.appliedVolts = intakeRollersSim.getInputVoltage();
       super.currentAmps = intakeRollersSim.getCurrentDrawAmps();
       super.velocity = intakeRollersSim.getAngularVelocityRPM() / 60.0;
       super.desiredVelocity = desiredVelocity;
       super.tempCelsius = 25.0;

       SimLog.log("intakeRollersSimMotor", intakeRollersSim);
       DogLog.log("intakeRollers/VelocitySetpoint", desiredVelocity);
     }

}
