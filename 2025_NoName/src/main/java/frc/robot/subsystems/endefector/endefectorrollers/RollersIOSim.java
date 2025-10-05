package frc.robot.subsystems.endefector.endefectorrollers;

import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.gearRatio;
import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.rollersMOI;
import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.simkD;
import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.simkI;
import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.simkP;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.EndefectorRollerStates;
import frc.robot.util.SubsystemUtil;

public class RollersIOSim extends RollersIO {
  private final DCMotorSim rollersSim;
  private PIDController rollersController = new PIDController(simkP, simkI, simkD);
  private static final DCMotor ROLLERS_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public RollersIOSim() {
    rollersSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ROLLERS_GEARBOX, rollersMOI, gearRatio),
            ROLLERS_GEARBOX);
    rollersController =
        new PIDController(RollersConstants.simkP, RollersConstants.simkI, RollersConstants.simkD);
  }

  @Override
  public void updateInputs() {
    rollersSim.update(0.02);

    super.appliedVolts = rollersSim.getInputVoltage();
    super.statorCurrentAmps = rollersSim.getCurrentDrawAmps();
    super.velocity = rollersSim.getAngularVelocityRPM() / 60.0;
    super.tempCelsius = 25.0;

    DogLog.log("Rollers/VelocitySetpoint", desiredVelocity);
    DogLog.log("Rollers/Velocity", super.velocity);
    DogLog.log("Rollers/Voltage", super.appliedVolts);
    DogLog.log("Rollers/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Rollers/Temp", 60);

    DogLog.log("Rollers/CoralInRamp", super.isCoralInRamp);
    DogLog.log("Rollers/CoralIntakedInEndefector", super.isCoralIntakedInEndefector);
    DogLog.log("Rollers/AlgaeIntaked", super.isReefAlgaeIntaked);
    DogLog.log("Rollers/CoralTroughScored", super.isCoralTroughScored);
    DogLog.log("Rollers/CoralBranchScored", super.isCoralBranchScored);
    DogLog.log("Rollers/AlgaeScored", super.isAlgaeScored);
  }

  @Override
  public void stop() {
    rollersSim.setAngularVelocity(0);
  }

  @Override
  public void setVelocity(EndefectorRollerStates state) {
    double velocity = SubsystemUtil.rollersStateToVelocity(state);
    rollersSim.setAngularVelocity(velocity);
  }

  @Override
  public void holdAlgae() {
    rollersSim.setInputVoltage(RollersConstants.rollersDutyCycleOutHoldAlgae);
  }

  @Override
  public void setCoralStateSim(boolean state) {
    super.isCoralInRamp = state;
    super.isCoralIntakedInEndefector = state;
  }
}
