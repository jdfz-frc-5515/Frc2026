// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// this file is used for a long-time project in 5515, which will not be used in competitio
// thus, it will not affect any functionality of the robot and will not be triggered unless stated explicitly
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/** An example command that uses an example subsystem. */
public class PowerOptBurnParamCmd extends Command {
    double newkP = 1.0;
    double newkI= 0.0;
    double newkD = 0.0;
    double newkS = 1.0;
    double newkV= 0.0;
    double newkA = 0.0;

    // direction: 0 up 1 left 2 down 3 right
    public PowerOptBurnParamCmd() {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Slot0Configs newGains = new Slot0Configs()
        .withKP(newkP).withKI(newkI).withKD(newkD)
        .withKS(newkS).withKV(newkV).withKA(newkA)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        TalonFX motor1 = new TalonFX(1, TunerConstants.kCANBus);
        TalonFX motor2 = new TalonFX(3, TunerConstants.kCANBus);
        TalonFX motor3 = new TalonFX(5, TunerConstants.kCANBus);
        TalonFX motor4 = new TalonFX(7, TunerConstants.kCANBus);
        motor1.getConfigurator().apply(newGains);
        motor2.getConfigurator().apply(newGains);
        motor3.getConfigurator().apply(newGains);
        motor4.getConfigurator().apply(newGains);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
