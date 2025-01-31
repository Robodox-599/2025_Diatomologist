package frc.robot.subsystems.algaegroundintake.intakeRollers;

import static frc.robot.subsystems.algaegroundintake.intakeRollers.IntakeRollersConstants.*;
// import static frc.robot.subsystems.algaegroundintake.wrist.WristIOSim.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.robot.util.SimLog;
//import frc.robot.subsystems.algaegroundintake.wrist.WristIOSim;


public class IntakeRollersIOSim extends IntakeRollersIO {
        private final DCMotor rollerGearbox = DCMotor.getKrakenX60Foc(1);
        private PIDController controller = new PIDController(0, 0, 0);
        private SimpleMotorFeedforward ff;
        private final DCMotorSim rollersMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(rollerGearbox, rollersMOI, gearRatio), rollerGearbox);

        
        private double appliedVoltage;
        private double desiredSpeed;

        public IntakeRollersIOSim() {

        }
    @Override
    public void updateInputs() {
        rollersMotorSim.update(0.2);

        super.appliedVolts = rollersMotorSim.getInputVoltage();
        super.currentAmps = rollersMotorSim.getCurrentDrawAmps();
        super.velocity = rollersMotorSim.getAngularVelocityRPM() / 60.0;
        super.desiredVelocity = desiredVelocity;
        super.tempCelsius = 25.0;

        SimLog.log("rollers", rollersMotorSim);
        DogLog.log("Rollers/VelocitySetpoint", desiredVelocity);
        
    }
    @Override
    public void setVoltage(double volts) {
        appliedVoltage = volts;
        rollersMotorSim.setInputVoltage(volts);
    }

        @Override
        public void setVelocity(double velocity) {
            double volts = controller.calculate(rollersMotorSim.getAngularVelocityRadPerSec(), velocity) + ff.calculate(simVelocityConstant);
            rollersMotorSim.setInputVoltage(volts);
        }

    @Override 
    public void setBrake(boolean brake) {
        if (brake) {
            appliedVoltage = 0;
            rollersMotorSim.setInputVoltage(0);
        }
    }
}
