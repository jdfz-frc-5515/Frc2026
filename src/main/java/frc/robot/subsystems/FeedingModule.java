// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class FeedingModule {

    private final TalonFX m_pathMotor = new TalonFX(Constants.PathMotor.motorID, new CANBus(Constants.PathMotor.canBusName));
    private final TalonFX m_feedMotor = new TalonFX(Constants.FeedMotor.motorID, new CANBus(Constants.FeedMotor.canBusName));
    private VelocityVoltage m_feedMotorDutycycle = new VelocityVoltage(0);
    private PositionVoltage m_feedPositionControl = new PositionVoltage(0); // 新增位置控制请求
    private VelocityVoltage m_pathMotorDutycycle = new VelocityVoltage(0);
    private VoltageOut m_voltageRequest = new VoltageOut(0);
    private boolean m_isPathMotorOn = false;
    private boolean m_isFeedMotorOn = false;
    private boolean m_isReversing = false; // 新增：是否正在执行反转任务
    private double m_targetPosition = 0;   // 新增：反转的目标位置

    public FeedingModule() {
        init();
    }

    private TalonFXConfiguration getFeedMotorConfiguration() {
            TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        // elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -200;
        // elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.Slot0.kP = Constants.FeedMotor.KP;
        config.Slot0.kI = Constants.FeedMotor.KI;
        config.Slot0.kD = Constants.FeedMotor.KD;
        config.Slot0.kS = Constants.FeedMotor.KS;
        config.Slot0.kV = Constants.FeedMotor.KV;
        config.Slot0.kA = Constants.FeedMotor.KA;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        return config;
    }
    private TalonFXConfiguration getPathMotorConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        // elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -200;
        // elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.Slot0.kP = Constants.PathMotor.KP;
        config.Slot0.kI = Constants.PathMotor.KI;
        config.Slot0.kD = Constants.PathMotor.KD;
        config.Slot0.kS = Constants.PathMotor.KS;
        config.Slot0.kV = Constants.PathMotor.KV;
        config.Slot0.kA = Constants.PathMotor.KA;

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        return config;
    }

    public void init() {
        m_feedMotor.getConfigurator().apply(getFeedMotorConfiguration());
        m_pathMotor.getConfigurator().apply(getPathMotorConfiguration());
    }

    int startTime = -1;
    int startDelayTime = 20;
    public void startFeedMotor() {
        // m_isFeedMotorOn = true;
        m_isReversing = false; // 手动启动喂料时，打断反转任务
        startTime = startDelayTime;
    }

    public void stopFeedMotor() {
        m_isReversing = false; 
        m_isFeedMotorOn = false;
    }

    public void startPathMotor() {
        m_isPathMotorOn = true;
    }

    public void stopPathMotor() {
        m_isPathMotorOn = false;
        startTime = -1;
    }

    public void reverseFeedMotor(double rotations) {
        m_isReversing = true;
        m_isFeedMotorOn = false; // 确保不会冲突
        // 计算目标位置：当前位置 - 要求的圈数
        m_targetPosition = m_feedMotor.getPosition().getValueAsDouble() - rotations;
    }


    public void update() {
        // --- 1. 处理反转逻辑 (优先级最高) ---
        if (m_isReversing) {
            m_feedMotor.setControl(m_feedPositionControl.withPosition(m_targetPosition));
            
            // 检查是否到达目标位置（容差设为 0.05 圈，约 18度）
            double currentPos = m_feedMotor.getPosition().getValueAsDouble();
            if (Math.abs(currentPos - m_targetPosition) < 0.05) {
                m_isReversing = false;
                m_feedMotor.stopMotor(); // 到达后停止
            }
        } 
        else {
            if (startTime >= 0) {
                if (startTime > 0) {
                    startTime--;
                }
                if (startTime == 0) {
                    m_isFeedMotorOn = true;
                }
            }

            if (m_isFeedMotorOn) {
                // m_feedMotorDutycycle.Velocity = Constants.FeedMotor.speed;
                // m_feedMotor.setControl(m_feedMotorDutycycle);
                m_voltageRequest.Output = 10;
                m_feedMotor.setControl(m_voltageRequest);
            }
            else {
                    m_voltageRequest.Output = 0;

                    m_feedMotor.setControl(m_voltageRequest);

                // m_feedMotorDutycycle.Velocity = 0;
                // m_feedMotor.stopMotor();
            }
        }
        if (m_isPathMotorOn) {
            m_pathMotorDutycycle.Velocity = Constants.PathMotor.speed;
            m_pathMotor.setControl(m_pathMotorDutycycle);
            // m_voltageRequest.Output = Constants.PathMotor.voltage;
            // m_pathMotor.setControl(m_voltageRequest);
        }
        else {
            // m_pathMotorDutycycle.Velocity = 0;
            // m_pathMotor.stopMotor();
            m_voltageRequest.Output = 0;
            m_pathMotor.setControl(m_voltageRequest);
        }
    }
}
