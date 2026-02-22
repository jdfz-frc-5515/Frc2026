// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeedingConstants;

public class FeedingSubsystem extends SubsystemBase {

    private final TalonFX m_pathMotor = new TalonFX(Constants.PathMotor.motorID, new CANBus(Constants.PathMotor.canBusName));
    private final TalonFX m_feedMotor = new TalonFX(Constants.FeedMotor.motorID, new CANBus(Constants.FeedMotor.canBusName));
    private VelocityVoltage m_feedMotorDutycycle = new VelocityVoltage(0);
    private VelocityVoltage m_pathMotorDutycycle = new VelocityVoltage(0);
    private boolean m_isPathMotorOn = false;
    private boolean m_isFeedMotorOn = false;

    public FeedingSubsystem() {
        init();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        update();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
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

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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

    public void startFeedMotor() {
        m_isFeedMotorOn = true;
    }

    public void stopFeedMotor() {
        m_isFeedMotorOn = false;
    }

    public void startPathMotor() {
        m_isPathMotorOn = true;
    }

    public void stopPathMotor() {
        m_isPathMotorOn = false;
    }

    public void update() {
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
