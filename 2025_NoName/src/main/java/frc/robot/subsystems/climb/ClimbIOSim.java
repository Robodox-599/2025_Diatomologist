// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.ClimbUtil;
import frc.robot.util.SimLog;

public class ClimbIOSim extends ClimbIO {
  private final DCMotorSim climbSim;
  private final PIDController positionController;
  private double targetPositionInches = 0.0;
  // private boolean brakeMode = true;
  private final PIDController simPidController =
      new PIDController(ClimbConstants.simkP, ClimbConstants.simkI, ClimbConstants.simkD);

  private static final DCMotor CLIMB_GEARBOX = DCMotor.getKrakenX60Foc(2);

  public ClimbIOSim() {
    climbSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                CLIMB_GEARBOX, ClimbConstants.climbMOI, ClimbConstants.gearRatio),
            CLIMB_GEARBOX);

    positionController =
        new PIDController(ClimbConstants.simkP, ClimbConstants.simkI, ClimbConstants.simkD);
    positionController.setTolerance(ClimbConstants.PositionToleranceInches);
  }

  @Override
  public void updateInputs() {
    climbSim.update(0.02);

    // Update inputs structure
    super.positionInches = climbSim.getAngularPositionRad() * ClimbConstants.inchesPerRev;
    super.velocityInchesPerSec =
        climbSim.getAngularAccelerationRadPerSecSq() * ClimbConstants.inchesPerRev;
    super.appliedVolts = climbSim.getCurrentDrawAmps();
    super.currentAmps = climbSim.getCurrentDrawAmps();
    super.targetPositionInches = targetPositionInches;
    super.tempCelsius = 25.0; // setting

    climbSim.setInputVoltage(
        positionController.calculate(super.positionInches, super.targetPositionInches));

    /* Checks if elevator is at setpoint */
    super.atSetpoint = positionController.atSetpoint();

    // Update state
    SimLog.log("climbMotors", climbSim);

    DogLog.log("Climb/PositionInches", super.positionInches);
    DogLog.log("Climb/VelocityInchesPerSec", super.velocityInchesPerSec);
    DogLog.log("Climb/AppliedVolts", super.appliedVolts);
    DogLog.log("Climb/TargetPositionInches", super.targetPositionInches);
    DogLog.log("Climb/AtSetpoint", super.atSetpoint);
    DogLog.log("Climb/State", super.state.toString());

    // System.out.println(super.positionInches);
  }

  @Override
  public void setState(ClimbConstants.ClimbStates state) {
    targetPositionInches =
        MathUtil.clamp(
            ClimbUtil.stateToHeight(state),
            ClimbConstants.climbLowerLimit,
            ClimbConstants.climbUpperLimit);

    climbSim.setInputVoltage(simPidController.calculate(targetPositionInches));
  }

  @Override
  public void stop() {
    climbSim.setInputVoltage(0);
  }

  @Override
  public void setVoltage(double voltage) {
    climbSim.setInputVoltage(voltage);
  }

  @Override
  public void zeroEncoder() {
    climbSim.setAngle(0);
  }

  @Override
  public double getPositionInches() {
    return climbSim.getAngularPositionRad() * ClimbConstants.inchesPerRev;
  }
}
