package frc.robot.subsystems.rollers;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.SimLog;

public class RollersIOSim implements RollersIO {
  private final DCMotorSim rollersSim;
  private static final DCMotor ROLLERS_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private PIDController rollerController =
      new PIDController(RollerConstants.simkP, 0, RollerConstants.simkD);
  private double desiredVelo = 0.0;

  public RollersIOSim() {
    rollersSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ROLLERS_GEARBOX,
                RollerConstants.kRollerInertia,
                RollerConstants.kRollerMotorGearRatio),
            ROLLERS_GEARBOX);
  }

  @Override
  public void setVoltage(double voltage) {
    rollersSim.setInputVoltage(voltage);
  }

  @Override
  public void runVelocity(double velocity) {
    desiredVelo = velocity;
    rollersSim.setInputVoltage(
        rollerController.calculate(rollersSim.getAngularVelocityRPM() / 60.0, velocity));
  }

  @Override
  public void stop() {
    runVelocity(0);
  }

  @Override
  public void updateInputs() {
    SimLog.log("Rollers", rollersSim);
    DogLog.log("Rollers/VelocitySetpoint", desiredVelo);
  }
}
