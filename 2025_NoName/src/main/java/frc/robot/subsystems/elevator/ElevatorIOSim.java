package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.SubsystemUtil;

public class ElevatorIOSim extends ElevatorIO {
  private final PIDController positionController;
  private double targetPositionInches = 0.0;

  private static final DCMotor ELEVATOR_GEARBOX = DCMotor.getKrakenX60Foc(2);
  private static final ElevatorSim elevatorSim =
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
    positionController =
        new PIDController(
            ElevatorConstants.simkP, ElevatorConstants.simkI, ElevatorConstants.simkD);
    positionController.setTolerance(ElevatorConstants.positionToleranceInches);
  }

  @Override
  public void updateInputs() {
    elevatorSim.update(0.02);
    // super.velocityInchesPerSec =
    // (Units.metersToInches(elevatorSim.getVelocityMetersPerSecond()));
    super.appliedVolts = elevatorSim.getInput().get(0, 0);
    super.currentAmps = elevatorSim.getCurrentDrawAmps();
    super.targetPositionInches = targetPositionInches;
    super.tempCelsius = 25.0;
    super.atSetpoint = true;

    DogLog.log("Elevator/CurrentAmps", elevatorSim.getCurrentDrawAmps());
    DogLog.log("Elevator/AppliedVoltage", elevatorSim.getInput().get(0, 0));
    DogLog.log("Elevator/PositionInches", super.positionInches);
    // DogLog.log("Elevator/VelocityInchesPerSec", super.velocityInchesPerSec);
    DogLog.log("Elevator/TargetPositionInches", super.targetPositionInches);
    DogLog.log("Elevator/AtSetpoint", super.atSetpoint);
    DogLog.log("Elevator/SoftUpperLimit", super.elevatorSoftUpperLimit);
    DogLog.log("Elevator/SoftLowerLimit", super.elevatorSoftLowerLimit);
  }

  @Override
  public void setHeight(ElevatorConstants.ElevatorStates state) {
    targetPositionInches =
        MathUtil.clamp(
            SubsystemUtil.elevatorStateToHeightTicks(state),
            ElevatorConstants.elevatorHardLowerLimit,
            ElevatorConstants.elevatorHardUpperLimit);
    positionInches = targetPositionInches;
  }

  @Override
  public void stop() {
    elevatorSim.setInputVoltage(0);
  }

  @Override
  public void setVoltage(double voltage) {
    elevatorSim.setInputVoltage(voltage);
  }
}
