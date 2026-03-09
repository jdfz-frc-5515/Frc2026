// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class FeedingModule {

    private final TalonFX m_pathMotor = new TalonFX(Constants.PathMotor.motorID, new CANBus(Constants.PathMotor.canBusName));
    private final TalonFX m_feedMotor = new TalonFX(Constants.FeedMotor.motorID, new CANBus(Constants.FeedMotor.canBusName));
    private VelocityVoltage m_feedMotorDutycycle = new VelocityVoltage(0);
    private VelocityVoltage m_pathMotorDutycycle = new VelocityVoltage(0);
    private boolean m_isPathMotorOn = false;
    private boolean m_isFeedMotorOn = false;

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

        return config;
    }

    public void init() {
        m_feedMotor.getConfigurator().apply(getFeedMotorConfiguration());
        m_pathMotor.getConfigurator().apply(getPathMotorConfiguration());
    }

    int startTime = -1;
    int startDelayTime = 100;
    public void startFeedMotor() {
        // m_isFeedMotorOn = true;
        startTime = startDelayTime;
    }

    public void stopFeedMotor() {
        m_isFeedMotorOn = false;
    }

    public void startPathMotor() {
        m_isPathMotorOn = true;
    }

    public void stopPathMotor() {
        m_isPathMotorOn = false;
        startTime = -1;
    }

    public void update() {
        if (startTime >= 0) {
            if (startTime > 0) {
                startTime--;
            }
            if (startTime == 0) {
                m_isFeedMotorOn = true;
            }
        }

        if (m_isFeedMotorOn) {
            m_feedMotorDutycycle.Velocity = Constants.FeedMotor.speed;
            m_feedMotor.setControl(m_feedMotorDutycycle);
        }
        else {
            m_feedMotorDutycycle.Velocity = 0;
            m_feedMotor.stopMotor();
        }

        if (m_isPathMotorOn) {
            m_pathMotorDutycycle.Velocity = Constants.PathMotor.speed;
            m_pathMotor.setControl(m_pathMotorDutycycle);
        }
        else {
            m_pathMotorDutycycle.Velocity = 0;
            m_pathMotor.stopMotor();
        }
        
    }
}
