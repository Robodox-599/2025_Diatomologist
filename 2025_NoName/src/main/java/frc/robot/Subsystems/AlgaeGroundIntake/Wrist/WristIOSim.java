package frc.robot.subsystems.algaegroundintake.wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
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
}
