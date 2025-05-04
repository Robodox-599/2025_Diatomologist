package frc.robot.subsystems.endefector.endefectorrollers;

import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.gearRatio;
import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.rollersDutyCycleOutHoldAlgae;
import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.rollersMOI;
import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.simkD;
import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.simkI;
import static frc.robot.subsystems.endefector.endefectorrollers.RollersConstants.simkP;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.SubsystemUtil;

public class RollersIOSim extends RollersIO {
  private final DCMotorSim rollersSim;
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
    super.statorCurrentAmps = rollersSim.getCurrentDrawAmps();
    super.velocity = rollersSim.getAngularVelocityRPM() / 60.0;
    super.tempCelsius = 25.0;

    switch (super.currentState) {
      case HOLDALGAE:
        rollersSim.setInputVoltage(rollersDutyCycleOutHoldAlgae * 12);
        break;
      case ADJUSTCORALAFTERSTATIONINTAKE:
        rollersSim.setInputVoltage(
            rollerController.calculate(
                rollersSim.getAngularPositionRotations(),
                rollersSim.getAngularPositionRotations()
                    + RollersConstants.rotationsToMoveAfterDetectingCoral));
        break;
      default:
        super.desiredVelocity = SubsystemUtil.rollersStateToVelocity(super.currentState) * 5800;
        rollersSim.setInputVoltage(
            rollerController.calculate(
                super.velocity, SubsystemUtil.rollersStateToVelocity(currentState) * 5800));
        break;
    }
    DogLog.log("Rollers/VelocitySetpoint", desiredVelocity);
    DogLog.log("Rollers/State", super.currentState);
    DogLog.log("Rollers/Velocity", super.velocity);
    DogLog.log("Rollers/Voltage", super.appliedVolts);
    DogLog.log("Rollers/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Rollers/Temp", 60);
  }

  @Override
  public void stop() {
    setVelocity(0);
  }

  @Override
  public void setState(RollersConstants.EndefectorRollerStates state) {
    super.currentState = state;
  }
}
