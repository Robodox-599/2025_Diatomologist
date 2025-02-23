package frc.robot.subsystems.endefector.endefectorrollers;

import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.EndefectorUtil;

public class RollersIOSim extends RollersIO {
  private final DCMotorSim rollersSim;
  private double desiredVelocity;
  private PIDController rollerController = new PIDController(simkP, simkI, simkD);
  private static final DCMotor ROLLERS_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public RollersIOSim() {
    rollersSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ROLLERS_GEARBOX, rollersMOI, gearRatio),
            ROLLERS_GEARBOX);
  }

  @Override
  public void updateInputs() {
    rollersSim.update(0.02);

    super.appliedVolts = rollersSim.getInputVoltage();
    super.currentAmps = rollersSim.getCurrentDrawAmps();
    super.velocity = rollersSim.getAngularVelocityRPM() / 60.0;
    super.desiredVelocity = desiredVelocity;
    super.tempCelsius = 25.0;

    rollersSim.setInputVoltage(rollerController.calculate(super.velocity, super.desiredVelocity));

    DogLog.log("Rollers/VelocitySetpoint", desiredVelocity);
    DogLog.log("Rollers/State", super.currentState);
    DogLog.log("Rollers/Velocity", super.velocity);
    DogLog.log("Rollers/Voltage", super.appliedVolts);
    DogLog.log("Rollers/Amps", super.currentAmps);
    DogLog.log("Rollers/Temp", 60);
  }

  @Override
  public void setVoltage(double voltage) {
    rollersSim.setInputVoltage(voltage);
  }

  @Override
  public void setVelocity(double velocity) {
    desiredVelocity = velocity;
    rollersSim.setInputVoltage(rollerController.calculate(velocity, desiredVelocity));
  }

  @Override
  public void stop() {
    setVelocity(0);
  }

  @Override
  public void setState(RollersConstants.EndefectorRollerStates state) {
    super.currentState = state;

    desiredVelocity = EndefectorUtil.stateToVelocity(state);

    rollersSim.setInputVoltage(rollerController.calculate(desiredVelocity));
    System.out.println(super.velocity);
  }

  @Override
  public double getCoralDistance() {
    return Math.random() * 10;
  }
}
