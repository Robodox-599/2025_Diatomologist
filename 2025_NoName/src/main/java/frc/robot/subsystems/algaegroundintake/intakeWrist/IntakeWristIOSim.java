package frc.robot.subsystems.algaegroundintake.intakeWrist;
import static frc.robot.subsystems.algaegroundintake.intakeWrist.IntakeWristConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.util.SimLog;

public class IntakeWristIOSim extends IntakeWristIO {
    private final DCMotor wristGearbox = DCMotor.getKrakenX60Foc(1);
    private DCMotorSim wristSim;
    
        private double passedInPosition;
        private double currentPosition;

    
        private ProfiledPIDController m_controller;
        private SimpleMotorFeedforward m_Feedforward = new  SimpleMotorFeedforward(0, 0);
    
        private SingleJointedArmSim sim =
            new SingleJointedArmSim(wristGearbox, IntakeWristSimConstants.kArmReduction, SingleJointedArmSim.estimateMOI(IntakeWristSimConstants.kArmLength,
            IntakeWristSimConstants.kArmMass),
            IntakeWristSimConstants.kArmLength,
            IntakeWristSimConstants.kMinAngleRads,
            IntakeWristSimConstants.kMaxAngleRads,
            true,
            0.1);
    
            private EncoderSim m_encoderSim;
    
            public void WristIOSim() {
                m_encoderSim = 
                    new EncoderSim(new Encoder(IntakeWristSimConstants.kEncoderAChannel, 
                    IntakeWristSimConstants.kEncoderBChannel));
    
            m_encoderSim.setDistancePerPulse(IntakeWristSimConstants.kArmEncoderDistPerPulse);
    
            m_controller = 
                new ProfiledPIDController(
                    IntakeWristSimConstants.kPivotSimPID[0], 
                    IntakeWristSimConstants.kPivotSimPID[1], 
                    IntakeWristSimConstants.kPivotSimPID[2],
                    new TrapezoidProfile.Constraints(2.45, 2.45)
                );
    
            m_controller.setTolerance(0.1, 0.05);
    
            wristSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                wristGearbox, wristMOI, gearRatio), wristGearbox);
        

        }

        @Override 
        public void updateInputs() {
            wristSim.update(0.02);

            super.appliedVolts = wristSim.getInputVoltage();
            super.currentAmps = wristSim.getCurrentDrawAmps();
            super.velocity = wristSim.getAngularVelocityRPM() / 60.0;
            super.targetPosition = passedInPosition;
            super.currentPosition = currentPosition;
            super.position = wristSim.getAngularPositionRotations();
            super.tempCelsius = 25.0;

            SimLog.log("WristSimMotor", wristSim);
            
            DogLog.log("Wrist/TargetPosition", passedInPosition);
            DogLog.log("Wrist/CurrentPosition", currentPosition);
            DogLog.log("Wrist/Position", super.position);       
        } 

        @Override 
        public void setVoltage(double volts) {
            sim.setInputVoltage(volts);
        }

        @Override 
        public void goToSetpoint(double setpoint) {
            m_controller.setGoal(setpoint);
            double pidOutput = m_controller.calculate(getAngle());
            double feedfowardOutput = m_Feedforward.calculate(m_controller.getSetpoint().velocity);

            sim.setInputVoltage(pidOutput + feedfowardOutput);
        }

        @Override 
        public double getAngle() {
                return sim.getAngleRads();
        }

        @Override
        public boolean atSetpoint() {
            return m_controller.atGoal();
        }

        @Override
        public void setP(double p) {
            m_controller.setP(p);
        }

        @Override
        public void setI(double i) {
            m_controller.setI(i);
        }

        @Override
        public void setD(double d) {
            m_controller.setD(d);
        }

        @Override 
        public void setkS(double kS) {
            m_Feedforward = new SimpleMotorFeedforward(kS, m_Feedforward.getKv());  
        }

        @Override 
        public void setkV(double kV) {
            m_Feedforward = new SimpleMotorFeedforward (m_Feedforward.getKs(), kV);
        }

        @Override 
        public double getP() {
            return m_controller.getP();
        }

        @Override
        public double getI() {
            return m_controller.getI();
        }

        @Override 
        public double getD() {
            return m_controller.getD();
        }

        @Override 
        public double getkS() {
            return m_Feedforward.getKs();
        }

        @Override
        public double getkV() {
            return m_Feedforward.getKv();
        }
}
