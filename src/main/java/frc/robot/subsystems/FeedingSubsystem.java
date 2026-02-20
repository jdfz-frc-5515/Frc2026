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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeedingConstants;

public class FeedingSubsystem extends SubsystemBase {
  // Feeding constants are centralized in Constants.FeedingConstants
  private final TalonFX feedMotor;
  private final TalonFX pathMotor;
  private final CANBus feedingSubsysCanBus = new CANBus("rio");
  private final VelocityVoltage feedControl;
  private final DutyCycleOut pathControl;
  private final TalonFXConfiguration feedConfig;
  /** Creates a new ExampleSubsystem. */
  public FeedingSubsystem() {
  feedMotor = new TalonFX(FeedingConstants.FEED_MOTOR_CAN_ID, feedingSubsysCanBus);
  pathMotor = new TalonFX(FeedingConstants.PATH_MOTOR_CAN_ID, feedingSubsysCanBus);
    feedConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
      .withSlot0(
        new Slot0Configs()
          .withKP(FeedingConstants.KP)
          .withKI(FeedingConstants.KI)
          .withKD(FeedingConstants.KD)
          .withKS(FeedingConstants.KS)
          .withKV(FeedingConstants.KV)
          .withKA(FeedingConstants.KA)
      );
  feedMotor.getConfigurator().apply(feedConfig);
  feedControl = new VelocityVoltage(0);
  pathControl = new DutyCycleOut(0);
  feedMotor.setControl(feedControl.withVelocity(FeedingConstants.FEED_STATIC_VELOCITY));
  pathMotor.setControl(pathControl.withOutput(FeedingConstants.PATH_STATIC_POWER));
  }

  public Command startFeedingCommand() {
    return edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          feedMotor.setControl(feedControl.withVelocity(FeedingConstants.FEED_SPIN_VELOCITY));
          pathMotor.setControl(pathControl.withOutput(FeedingConstants.PATH_SPIN_POWER));
        },
        () -> {
          feedMotor.setControl(feedControl.withVelocity(FeedingConstants.FEED_STATIC_VELOCITY));
          pathMotor.setControl(pathControl.withOutput(FeedingConstants.PATH_STATIC_POWER));
        },
        this);
  }

  public Command stopFeedingCommand() {
    return new InstantCommand(
        () -> {
          feedMotor.setControl(feedControl.withVelocity(FeedingConstants.FEED_STATIC_VELOCITY));
          pathMotor.setControl(pathControl.withOutput(FeedingConstants.PATH_STATIC_POWER));
        });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("feedMotorVel", feedMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("pathMotorVel", pathMotor.getVelocity().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
