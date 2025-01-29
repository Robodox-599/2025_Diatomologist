package frc.robot.subsystems.algaegroundintake.rollers;

import static frc.robot.subsystems.algaegroundintake.rollers.RollersConstants.*;
import static frc.robot.subsystems.algaegroundintake.wrist.WristIOSim.*;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.robot.subsystems.algaegroundintake.utils.SimLog;
import frc.robot.subsystems.algaegroundintake.wrist.WristIOSim;


public class RollersIOSim extends RollersIO {
        private final DCMotor rollerGearbox = DCMotor.getKrakenX60Foc(1);
        private PIDController controller = new PIDController(0, 0, 0);
        private SimpleMotorFeedforward ff;
        private final DCMotorSim rollersMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(rollerGearbox, RollersConstants.rollersMOI, RollersConstants.gearRatio), rollerGearbox);

        
        private double appliedVoltage;
        private double desiredSpeed;

        public RollersIOSim() {

        }
    @Override
    public void updateInputs() {
        rollersMotorSim.update(0.2);
        SimLog.log("rollers", rollerGearbox);

        
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
