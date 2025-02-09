package frc.robot.subsystems.algaegroundintake.intakewrist;

import static frc.robot.subsystems.algaegroundintake.intakewrist.IntakeWristConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.AlgaeGroundIntakeUtil;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.util.SimLog;

public class IntakeWristIOSim extends IntakeWristIO {
  private static final DCMotor WRIST_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim wristSim;

  private double passedInPosition;
  private double currentPosition;
  private int wristSlot;

  private final PIDController wristPID =
      new PIDController(
          IntakeWristConstants.simkP, IntakeWristConstants.simkI, IntakeWristConstants.simkD);

  public IntakeWristIOSim() {
    wristSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(WRIST_GEARBOX, wristMOI, gearRatio), WRIST_GEARBOX);
  }

  @Override
  public void updateInputs() {
    wristSim.update(0.02);

    super.appliedVolts = wristSim.getInputVoltage();
    super.currentAmps = wristSim.getCurrentDrawAmps();
    super.velocity = wristSim.getAngularVelocityRPM() / 60.0;
    super.targetPosition = targetPosition;
    super.currentPosition = currentPosition;
    super.position = wristSim.getAngularPositionRotations();
    super.tempCelsius = 25.0;

    SimLog.log("WristSimMotor", wristSim);

    DogLog.log("Wrist/TargetPosition", passedInPosition);
    DogLog.log("Wrist/CurrentPosition", currentPosition);
    DogLog.log("Wrist/Position", super.position);
  }

  @Override
  public void goToPose(double position) {
    wristSim.setInputVoltage(wristPID.calculate(position));
  }

  @Override
  public void setVoltage(double voltage) {
    wristSim.setInputVoltage(voltage);
  }

  @Override
  public double getPose() {
    return wristSim.getAngularPositionRad();
  }

  @Override
  public void stop() {
    wristSim.setAngularVelocity(0);
  }

  @Override
  public void setState(IntakeWristConstants.AlgaeStates state) {
    double position =
        MathUtil.clamp(
            AlgaeGroundIntakeUtil.stateToSetpoint(state),
            IntakeWristConstants.intakeWristLowerLimit,
            IntakeWristConstants.intakeWristUpperLimit);

    if (passedInPosition > currentPosition) {
      wristSlot = 0;
    } else {
      wristSlot = 1;
    }
  }
}
