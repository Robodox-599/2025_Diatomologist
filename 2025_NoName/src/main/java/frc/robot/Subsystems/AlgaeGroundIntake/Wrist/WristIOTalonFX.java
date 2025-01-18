package frc.robot.subsystems.algaegroundintake.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.algaegroundintake.utils.MotorLog;
import frc.robot.subsystems.algaegroundintake.wrist.WristConstants.IntakeWristSimConstants;

public class WristIOTalonFX implements WristIO {
    
    private final TalonFX intakeWristMotor = new TalonFX(WristConstants.wristMotorID, WristConstants.wristMotorCANBus);

    private double setPoint = 0.0;
    private double motorEncoder;
    private int m_WristSlot = 0;

    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<AngularVelocity> angleVelocityRadsPerSec;
    private final StatusSignal<Temperature> tempCelcius;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Angle> angleRads;

    public WristIOTalonFX() {
        var config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = WristConstants.wristExtendKP;
        config.Slot0.kI = WristConstants.wristExtendKI;
        config.Slot0.kD = WristConstants.wristExtendKD;
        config.Slot0.kS = WristConstants.wristExtendKS;

        config.Slot1.kP = WristConstants.wristRetractKP;
        config.Slot1.kI = WristConstants.wristRetractKI;
        config.Slot1.kD = WristConstants.wristRetractKD;

        var mmConfig = config.MotionMagic;
        mmConfig.MotionMagicCruiseVelocity = WristConstants.maxWristVelocity;
        mmConfig.MotionMagicAcceleration = WristConstants.maxWristAccel;

        intakeWristMotor.getConfigurator().apply(config);
        intakeWristMotor.setPosition(0);
        setBrake(true);

        motorEncoder = intakeWristMotor.getPosition().getValueAsDouble();
        appliedVolts =intakeWristMotor.getSupplyVoltage();
        angleVelocityRadsPerSec = intakeWristMotor.getVelocity();
        tempCelcius = intakeWristMotor.getDeviceTemp();
        angleRads = intakeWristMotor.getPosition();
        currentAmps = intakeWristMotor.getSupplyCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVolts, tempCelcius, angleRads, currentAmps);
        intakeWristMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        BaseStatusSignal.refreshAll(appliedVolts, tempCelcius, angleRads, currentAmps);
        inputs.setpointAngleRads = setPoint;
        inputs.appliedVoltage = appliedVolts.getValueAsDouble();
        inputs.tempCelcius = tempCelcius.getValueAsDouble();
        inputs.angularRads = angleRads.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();

        MotorLog.log("IntakeWristMotor", intakeWristMotor);
    }

    @Override 
    public void setVoltage(double motorVolts) {
        intakeWristMotor.setVoltage(motorVolts);
    }

    @Override
    public double getAngle() {
        return (Units.radiansToRotations(motorEncoder));
    }

    public void desiredWristSetPos(double passedInPosition) {
        m_WristSlot = 
            passedInPosition == WristConstants.kWristExtendVal
            ? WristConstants.wristExtendSlot
            : WristConstants.wristRetractSlot;
        setPoint = passedInPosition;
        MotionMagicVoltage m_request = 
            new MotionMagicVoltage(setPoint).withSlot(m_WristSlot).withFeedForward(WristConstants.kWristFeedForward);
    }

    @Override 
    public void goToSetpoint(double setPoint) {
        desiredWristSetPos(setPoint);
    }

    @Override
    public void holdToSetpoint(double setPoint) {
        goToSetpoint(setPoint);
    }

    @Override
    public void setBrake(boolean brake) {
        if(brake) {
            intakeWristMotor.setNeutralMode(NeutralModeValue.Brake);
        }
    }

    @Override 
    public boolean atSetpoint() {
        return Math.abs(getAngle() - setPoint) < WristConstants.intakeWristPositionTolerance;
    }
}
