// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.ClimbUtil;

public class ClimbIOSim extends ClimbIO {
  private final DCMotorSim climbSim;
  private final PIDController positionController;
  private double targetPositionInches = 0.0;

  private static final DCMotor CLIMB_GEARBOX = DCMotor.getKrakenX60Foc(2);
  private static final ElevatorSim simClimbTest =
      new ElevatorSim(
          CLIMB_GEARBOX,
          6,
          Units.lbsToKilograms(28),
          Units.inchesToMeters(1),
          Units.inchesToMeters(0),
          Units.inchesToMeters(89),
          true,
          0);

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
    simClimbTest.update(0.02);

    // Update inputs structure
    super.positionInches = (Units.metersToInches(simClimbTest.getPositionMeters()));
    super.velocityInchesPerSec = (Units.metersToInches(simClimbTest.getVelocityMetersPerSecond()));

    super.appliedVolts = simClimbTest.getInput().get(0, 0);
    super.currentAmps = simClimbTest.getCurrentDrawAmps();
    super.targetPositionInches = targetPositionInches;
    super.tempCelsius = 25.0; // setting

    simClimbTest.setInputVoltage(
        positionController.calculate(super.positionInches, super.targetPositionInches));

    /* Checks if elevator is at setpoint */
    super.atSetpoint = positionController.atSetpoint();
    DogLog.log("Climb/PositionInches", super.positionInches);
    DogLog.log("Climb/VelocityInchesPerSecond", super.velocityInchesPerSec);
    DogLog.log("Climb/CurrentAmps", simClimbTest.getCurrentDrawAmps());
    DogLog.log("Climb/AppliedVoltage", simClimbTest.getInput().get(0, 0));

    // Update state
    // SimLog.log("elevatorMotors", simElevatorTest);
    DogLog.log("Climb/PositionErrorInches", super.targetPositionInches - super.positionInches);

    DogLog.log("Climb/PositionInches", super.positionInches);
    DogLog.log("Climb/VelocityInchesPerSec", super.velocityInchesPerSec);
    DogLog.log("Climb/TargetPositionInches", super.targetPositionInches);
    DogLog.log("Climb/AtSetpoint", super.atSetpoint);
    DogLog.log("Climb/State", super.state.toString());

    System.out.println(super.positionInches);
  }

  @Override
  public void setState(ClimbConstants.ClimbStates state) {
    targetPositionInches =
        MathUtil.clamp(
            ClimbUtil.stateToHeight(state),
            ClimbConstants.climbLowerLimit,
            ClimbConstants.climbUpperLimit);
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
