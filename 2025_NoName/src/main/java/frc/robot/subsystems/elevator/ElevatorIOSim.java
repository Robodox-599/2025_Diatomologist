package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.ElevatorUtil;

public class ElevatorIOSim extends ElevatorIO {
  private final DCMotorSim elevatorSim;
  // private final ElevatorSim =
  private final PIDController positionController;
  private double targetPositionInches = 0.0;

  private static final DCMotor ELEVATOR_GEARBOX = DCMotor.getKrakenX60Foc(2);
  private static final ElevatorSim simElevatorTest =
      new ElevatorSim(
          ELEVATOR_GEARBOX,
          6,
          Units.lbsToKilograms(28),
          Units.inchesToMeters(1),
          Units.inchesToMeters(0),
          Units.inchesToMeters(89),
          true,
          0);

  public ElevatorIOSim() {

    elevatorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ELEVATOR_GEARBOX, ElevatorConstants.elevatorMOI, ElevatorConstants.gearRatio),
            ELEVATOR_GEARBOX);

    positionController =
        new PIDController(
            ElevatorConstants.simkP, ElevatorConstants.simkI, ElevatorConstants.simkD);
    positionController.setTolerance(ElevatorConstants.PositionToleranceInches);
  }

  @Override
  public void updateInputs() {
    simElevatorTest.update(0.02);

    // Update inputs structure
    super.positionInches = (Units.metersToInches(simElevatorTest.getPositionMeters()));
    super.velocityInchesPerSec =
        (Units.metersToInches(simElevatorTest.getVelocityMetersPerSecond()));

    super.appliedVolts = simElevatorTest.getInput().get(0, 0);
    super.currentAmps = simElevatorTest.getCurrentDrawAmps();
    super.targetPositionInches = targetPositionInches;
    super.tempCelsius = 25.0; // setting

    simElevatorTest.setInputVoltage(
        positionController.calculate(super.positionInches, super.targetPositionInches));

    /* Checks if elevator is at setpoint */
    super.atSetpoint = positionController.atSetpoint();
    DogLog.log("Elevator/PositionInches", super.positionInches);
    DogLog.log("Elevator/VelocityInchesPerSecond", super.velocityInchesPerSec);
    DogLog.log("Elevator/CurrentAmps", simElevatorTest.getCurrentDrawAmps());
    DogLog.log("Elevator/AppliedVoltage", simElevatorTest.getInput().get(0, 0));

    // Update state
    // SimLog.log("elevatorMotors", simElevatorTest);
    DogLog.log("Elevaotor/PositionErrorInches", super.targetPositionInches - super.positionInches);

    DogLog.log("Elevator/PositionInches", super.positionInches);
    DogLog.log("Elevator/VelocityInchesPerSec", super.velocityInchesPerSec);
    DogLog.log("Elevator/TargetPositionInches", super.targetPositionInches);
    DogLog.log("Elevator/AtSetpoint", super.atSetpoint);
    DogLog.log("Elevator/State", super.state.toString());
  }

  @Override
  public void setState(ElevatorConstants.ElevatorStates state) {
    targetPositionInches =
        MathUtil.clamp(
            ElevatorUtil.stateToHeight(state),
            ElevatorConstants.elevatorLowerLimit,
            ElevatorConstants.elevatorUpperLimit);
    System.out.println(super.state);
  }

  @Override
  public void stop() {
    elevatorSim.setInputVoltage(0);
  }

  @Override
  public void setVoltage(double voltage) {
    elevatorSim.setInputVoltage(voltage);
  }

  @Override
  public void zeroEncoder() {
    elevatorSim.setAngle(0);
  }

  @Override
  public double getPosition() {
    return elevatorSim.getAngularPositionRad() * ElevatorConstants.inchesPerRev;
  }
}
