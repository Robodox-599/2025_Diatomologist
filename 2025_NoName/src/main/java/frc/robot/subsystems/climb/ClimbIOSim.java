package frc.robot.subsystems.climb;

// import dev.doglog.DogLog;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim extends ClimbIO {
  //   private final DCMotorSim climbSim;
  //   private final DCMotorSim rollersSim;
  //   private final PIDController positionController;
  //   private final PIDController simPidController =
  //       new PIDController(ClimbConstants.simkP, ClimbConstants.simkI, ClimbConstants.simkD);

  //   private static final DCMotor CLIMB_GEARBOX = DCMotor.getKrakenX60Foc(2);
  //   private static final DCMotor ROLLERS_GEARBOX = DCMotor.getKrakenX60Foc(1);

  public ClimbIOSim() {
    //     climbSim =
    //         new DCMotorSim(
    //             LinearSystemId.createDCMotorSystem(
    //                 CLIMB_GEARBOX, ClimbConstants.climbMOI, ClimbConstants.gearRatio),
    //             CLIMB_GEARBOX);
    //     rollersSim =
    //         new DCMotorSim(
    //             LinearSystemId.createDCMotorSystem(ROLLERS_GEARBOX, ClimbConstants.rollersMOI,
    // 0),
    //             ROLLERS_GEARBOX);

    //     positionController =
    //         new PIDController(ClimbConstants.simkP, ClimbConstants.simkI, ClimbConstants.simkD);
  }

  //   @Override
  //   public void updateInputs() {
  //     climbSim.update(0.02);
  //     super.climbPositionDegrees = climbSim.getAngularPositionRotations();
  //     super.climbVelocity = climbSim.getAngularVelocityRPM() / 60.0;
  //     super.climbAppliedVolts = climbSim.getCurrentDrawAmps();
  //     super.climbCurrentAmps = climbSim.getCurrentDrawAmps();
  //     super.targetPositionDegrees = targetPositionDegrees;
  //     super.climbTempCelsius = 25.0; // setting
  //     super.rollersVelocity = rollersSim.getAngularVelocityRPM() / 60.0;
  //     super.rollersAppliedVolts = rollersSim.getCurrentDrawAmps();
  //     super.rollersCurrentAmps = rollersSim.getCurrentDrawAmps();
  //     super.rollersTempCelsius = 25.0; // setting

  //     climbSim.setInputVoltage(
  //         positionController.calculate(super.climbPositionDegrees, super.targetPositionDegrees));

  //     super.atSetpoint = positionController.atSetpoint();

  //     DogLog.log("Climb/IsCageDetected", super.isCageDetected);
  //     DogLog.log("Climb/StatorCurrentAmps", super.climbCurrentAmps);
  //     DogLog.log("Climb/AppliedVoltage", super.climbAppliedVolts);
  //     DogLog.log("Climb/Velocity", super.climbVelocity);
  //     DogLog.log("Climb/Temperature", super.climbTempCelsius);
  //     DogLog.log("Climb/CurrentPosition", super.climbPositionDegrees);
  //     DogLog.log("Climb/TargetPositon", super.targetPositionDegrees);
  //     DogLog.log("Climb/Rollers/StatorCurrentAmps", super.rollersCurrentAmps);
  //     DogLog.log("Climb/Rollers/AppliedVoltage", super.rollersAppliedVolts);
  //     DogLog.log("Climb/Rollers/Velocity", super.rollersVelocity);
  //     DogLog.log("Climb/Rollers/Temperature", super.rollersTempCelsius);
  //   }

  //   // @Override
  //   // public void setClimbPosition(ClimbStates state) {
  //   //   targetPositionDegrees =
  //   //       MathUtil.clamp(
  //   //           ClimbConstants.setpoint[state.getIndex()],
  //   //           ClimbConstants.climbLowerLimit,
  //   //           ClimbConstants.climbUpperLimit);
  //   //   climbSim.setInputVoltage(simPidController.calculate(targetPositionDegrees));
  //   // }

  //   @Override
  //   public void setRollersVelocity(double velocity) {
  //     rollersSim.setAngularVelocity(velocity);
  //   }

  //   @Override
  //   public void stop() {
  //     setClimbVoltage(0);
  //     setRollersVelocity(0);
  //   }

  //   @Override
  //   public void setClimbVoltage(double voltage) {
  //     climbSim.setInputVoltage(voltage);
  //     rollersSim.setInputVoltage(voltage);
  //   }
}
