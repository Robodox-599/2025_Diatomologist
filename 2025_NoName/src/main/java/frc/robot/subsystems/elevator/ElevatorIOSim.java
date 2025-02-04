package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.ElevatorUtil;
import frc.robot.util.SimLog;

public class ElevatorIOSim extends ElevatorIO {
  private final DCMotorSim elevatorSim;
  private final PIDController positionController;
  private double targetPositionInches = 0.0;
  public double position;

  // private Mechanism2d mech = new Mechanism2d(60, 60);
  // private MechanismRoot2d root = mech.getRoot("Root", 30, 5);
  // private MechanismLigament2d elevator = root.append(new MechanismLigament2d("Elevator",
  // ElevatorConstants.heights[4], 90, 6, new Color8Bit(Color.kBlue)));
  // private MechanismLigament2d wrist = elevator.append(new MechanismLigament2d("wrist", 6, 90, 6,
  // new Color8Bit(Color.kPurple)));

  // Add the arm connected to the base
  //
  // private MechanismLigament2d arm = elevBase.append(new MechanismLigament2d("Arm", 2, 45, 6, new
  // Color8Bit(Color.kBlue))); // Blue diagonal arm

  private final PIDController simPidController =
      new PIDController(ElevatorConstants.simkP, ElevatorConstants.simkI, ElevatorConstants.simkD);

  private static final DCMotor ELEVATOR_GEARBOX = DCMotor.getKrakenX60Foc(2);

  public ElevatorIOSim() {
    // SmartDashboard.putData("elevatorMech2d", mech);

    elevatorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ELEVATOR_GEARBOX, ElevatorConstants.elevatorMOI, ElevatorConstants.gearRatio),
            ELEVATOR_GEARBOX);

    positionController =
        new PIDController(
            ElevatorConstants.simkP, ElevatorConstants.simkI, ElevatorConstants.simkD);
    positionController.setTolerance(
        ElevatorConstants.PositionToleranceInches, ElevatorConstants.velocityToleranceInchesPerSec);
  }

  @Override
  public void updateInputs() {
    elevatorSim.update(0.02);

    // Update inputs structure
    super.positionInches =
        (elevatorSim.getAngularPositionRad() * ElevatorConstants.inchesPerRev) / 39.37;
    super.velocityInchesPerSec =
        elevatorSim.getAngularAccelerationRadPerSecSq() * ElevatorConstants.inchesPerRev;
    super.appliedVolts = elevatorSim.getCurrentDrawAmps() * ElevatorConstants.nominal_voltage;
    super.currentAmps = elevatorSim.getCurrentDrawAmps();
    super.targetPositionInches = targetPositionInches;
    // super.positionRotations = elevatorSim.getAngularPositionRotations(); // might not need
    super.tempCelsius = 25.0; // setting

    /* Checks if elevator is at setpoint */
    super.atSetpoint = positionController.atSetpoint();

    // Update state
    SimLog.log("elevatorMotors", elevatorSim);

    DogLog.log("Elevator/PositionInches", super.positionInches);
    DogLog.log("Elevator/VelocityInchesPerSec", super.velocityInchesPerSec);
    DogLog.log("Elevator/AppliedVolts", super.appliedVolts);
    DogLog.log("Elevator/TargetPositionInches", super.targetPositionInches);
    DogLog.log("Elevator/AtSetpoint", super.atSetpoint);
    // DogLog.log("Elevator/PositionRotations", super.positionRotations); // might not need
    DogLog.log("Elevator/State", super.state.toString());
  }

  @Override
  public void setState(ElevatorConstants.ElevatorStates state) {
    position =
        MathUtil.clamp(
            ElevatorUtil.stateToHeight(state),
            ElevatorConstants.elevatorLowerLimit,
            ElevatorConstants.elevatorUpperLimit);
    System.out.println(super.state);
    elevatorSim.setInputVoltage(simPidController.calculate(position));

    switch (state) {
      case L1:
        position = ElevatorConstants.heights[0];
        // elevator.setLength(position);
        System.out.println(state);
        break;
      case L2:
        position = ElevatorConstants.heights[1];
        // elevator.setLength(position);
        System.out.println(state);
        break;
      case L3:
        position = ElevatorConstants.heights[2];
        // elevator.setLength(position);
        System.out.println(state);
        break;
      case L4:
        position = ElevatorConstants.heights[3];
        // elevator.setLength(position);
        System.out.println(state);
        break;
      case STOW:
        position = ElevatorConstants.heights[4]; // STOW
        System.out.println(state);
        break;
    }
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
