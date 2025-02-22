package frc.robot.subsystems.endefector.endefectorwrist;

import static frc.robot.subsystems.endefector.endefectorwrist.WristConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;
import frc.robot.util.EndefectorUtil;

public class WristIOSim extends WristIO {

  private static final DCMotor WRIST_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim wristSim;

  // private double passedInPositon;
  // private double currentPosition;
  private PIDController wristPID = new PIDController(simkP, simkI, simkD);

  public WristIOSim() {
    wristSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(WRIST_GEARBOX, wristMOI, gearRatio), WRIST_GEARBOX);

    wristPID = new PIDController(WristConstants.simkP, WristConstants.simkI, WristConstants.simkD);
    wristPID.setTolerance(WristConstants.wristPositionLimit);
  }

  @Override
  public void updateInputs() {
    wristSim.update(0.02);

    super.atSetpoint = wristPID.atSetpoint();
    super.appliedVolts = wristSim.getInputVoltage();
    super.currentAmps = wristSim.getCurrentDrawAmps();
    super.velocity = wristSim.getAngularVelocityRPM() / 60.0;
    super.targetPosition = targetPosition;
    super.currentPositionDegrees = wristSim.getAngularPositionRotations();
    super.tempCelsius = 25.0;

    DogLog.log("Wrist/CurrentAmps", super.currentAmps);
    DogLog.log("Wrist/AppliedVoltage", super.appliedVolts);
    DogLog.log("Wrist/TargetPosition", super.targetPosition);
    DogLog.log("Wrist/CurrentPosition", super.currentPositionDegrees);
    DogLog.log("Wrist/Temperature", super.tempCelsius);

    wristSim.setInputVoltage(
        wristPID.calculate(super.currentPositionDegrees, super.targetPosition));
  }

  @Override
  public void setState(WristStates state) {
    targetPosition =
        MathUtil.clamp(EndefectorUtil.stateToSetpoint(state), wristMinAngle, wristMaxAngle);
    System.out.println(super.state);
    wristSim.setInputVoltage(wristPID.calculate(targetPosition));
  }

  @Override
  public void setVoltage(double voltage) {
    wristSim.setInputVoltage(voltage);
  }

  @Override
  public double getCurrentPosition() {
    return wristSim.getAngularPositionRad();
  }

  @Override
  public void stop() {
    wristSim.setAngularVelocity(0);
  }
}