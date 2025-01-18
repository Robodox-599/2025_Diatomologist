package frc.robot.subsystems.algaegroundintake.wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {
    private final DCMotor wristGearbox = DCMotor.getKrakenX60Foc(1);

    private ProfiledPIDController m_controller;
    private SimpleMotorFeedforward m_Feedforward = new  SimpleMotorFeedforward(0, 0);

    private SingleJointedArmSim sim =
        new SingleJointedArmSim(wristGearbox, WristConstants.IntakeWristSimConstants.kArmReduction, SingleJointedArmSim.estimateMOI(WristConstants.IntakeWristSimConstants.kArmLength,
        WristConstants.IntakeWristSimConstants.kArmMass),
        WristConstants.IntakeWristSimConstants.kArmLength,
        WristConstants.IntakeWristSimConstants.kMinAngleRads,
        WristConstants.IntakeWristSimConstants.kMaxAngleRads,
        true,
        0.1);

        private EncoderSim m_encoderSim;

        public void WristIOSim() {
            m_encoderSim = 
                new EncoderSim(new Encoder(WristConstants.IntakeWristSimConstants.kEncoderAChannel, 
                WristConstants.IntakeWristSimConstants.kEncoderBChannel));

        m_encoderSim.setDistancePerPulse(WristConstants.IntakeWristSimConstants.kArmEncoderDistPerPulse);

        m_controller = 
            new ProfiledPIDController(
                WristConstants.IntakeWristSimConstants.kPivotSimPID[0], 
                WristConstants.IntakeWristSimConstants.kPivotSimPID[1], 
                WristConstants.IntakeWristSimConstants.kPivotSimPID[2],
                new TrapezoidProfile.Constraints(2.45, 2.45)
            );

        m_controller.setTolerance(0.1, 0.05);

        }

        @Override 
        public void updateInputs(WristIOInputs inputs) {
            sim.update(0.02);
            inputs.angularRads = getAngle();
            inputs.getAngularVelocityRadPerSec = sim.getVelocityRadPerSec();
            inputs.currentAmps = sim.getCurrentDrawAmps();
            inputs.setpointAngleRads = m_controller.getSetpoint().position;
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
