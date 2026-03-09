package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MiscUtils;

public class IntakeSubsystem2 extends SubsystemBase {
    private final CANBus canBus = new CANBus(Constants.IntakeConstants.canBusName);
    private final TalonFX m_intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_CAN_ID, canBus);
    private final TalonFX m_extenderMotor = new TalonFX(Constants.IntakeConstants.EXTENDER_CAN_ID, canBus);
    private MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0);

    private boolean m_isExtend = false;
    
    private final double NONE_POS = -9999;
    private final double threshold = 0.05;

    private TalonFXConfiguration getExtenerMotorConfiguration(boolean lockMotor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Slot0.kP = Constants.IntakeConstants.EXTENDER_KP;
        config.Slot0.kI = Constants.IntakeConstants.EXTENDER_KI;
        config.Slot0.kD = Constants.IntakeConstants.EXTENDER_KD;
        config.Slot0.kS = Constants.IntakeConstants.EXTENDER_KS;
        config.Slot0.kV = Constants.IntakeConstants.EXTENDER_KV;
        config.Slot0.kA = Constants.IntakeConstants.EXTENDER_KA;

        config.MotorOutput.NeutralMode = lockMotor ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.IntakeConstants.EXTENDER_MOTION_MAGIC_MAX_SPEED;
        config.MotionMagic.MotionMagicAcceleration = Constants.IntakeConstants.EXTENDER_MOTION_MAGIC_ACC;
        config.MotionMagic.MotionMagicJerk = Constants.IntakeConstants.EXTENDER_MOTION_MAGIC_JERK;

        /* Current Limiting */
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLowerLimit = 30;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.02;

        config.CurrentLimits.StatorCurrentLimit = 20;       // 这个值在限制扭矩输出
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return config;
    }

    private boolean isExtenderAtPos(double targetPos) {
        if (Math.abs(m_extenderMotor.getPosition().getValueAsDouble() - targetPos) < threshold) {
            return true;
        }

        return false;
    }

    private TalonFXConfiguration getIntakeMotorConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Slot0.kP = Constants.IntakeConstants.INTAKE_KP;
        config.Slot0.kI = Constants.IntakeConstants.INTAKE_KI;
        config.Slot0.kD = Constants.IntakeConstants.INTAKE_KD;
        config.Slot0.kS = Constants.IntakeConstants.INTAKE_KS;
        config.Slot0.kV = Constants.IntakeConstants.INTAKE_KV;
        config.Slot0.kA = Constants.IntakeConstants.INTAKE_KA;

        /* Current Limiting */
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLowerLimit = 30;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.02;
        return config;
    }

    public IntakeSubsystem2() {
        init();
    }

    public void zeroCC() {
        m_extenderMotor.setPosition(0);
    }
    public void init() {
        m_extenderMotor.getConfigurator().apply(getExtenerMotorConfiguration(true));
        m_intakeMotor.getConfigurator().apply(getIntakeMotorConfiguration());

        zeroCC();
    }

    public void update() {
        double extPos = 0;
        double intakeMotorSpeed = 0;
        if (m_isExtend) {
            // 展开
            extPos = Constants.IntakeConstants.OUT_POS;
            intakeMotorSpeed = Constants.IntakeConstants.INTAKE_SPEED;
        }
        else {
            // 收起
            extPos = 0;
            intakeMotorSpeed = 0;
        }


        if (MiscUtils.compareDouble(extPos, NONE_POS) || isExtenderAtPos(extPos)) {
            m_extenderMotor.stopMotor();
        }
        else {
            m_extenderMotor.setControl(m_motionMagicVoltage.withPosition(extPos).withSlot(0));
        }

        if (MiscUtils.compareDouble(intakeMotorSpeed, 0)) {
            m_intakeMotor.stopMotor();
        }
        else {
            m_intakeMotor.set(intakeMotorSpeed);
        }

        SmartDashboard.putNumber("Intake/Extender Pos", m_extenderMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        update();
    }

    public void toggleExtender() {
        m_isExtend = !m_isExtend;
    }
}
