// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// this file is used for a long-time project in 5515, which will not be used in competitio
// thus, it will not affect any functionality of the robot and will not be triggered unless stated explicitly

import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/**
 * 烧录电机PID参数命令
 * 
 * 使用方法：
 * - 传入增益列表 [kP, kI, kD, kS, kV, kA]
 * - 通过 Command.schedule() 执行
 */
public class PowerOptBurnParamCmd extends Command {
    
    private static final int[] DEVICE_IDS = {1, 3, 5, 7};
    
    private final List<Double> gains; // [kP, kI, kD, kS, kV, kA]
    
    private final TalonFX[] motors;


    /**
     * @param gains 增益列表 [kP, kI, kD, kS, kV, kA]
     */
    public PowerOptBurnParamCmd(List<Double> gains) {
        this.gains = Collections.unmodifiableList(gains);
        
        motors = new TalonFX[DEVICE_IDS.length];
        for (int i = 0; i < DEVICE_IDS.length; i++) {
            motors[i] = new TalonFX(DEVICE_IDS[i], TunerConstants.kCANBus);
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (gains.size() < 6) return;
        
        Slot0Configs newGains = new Slot0Configs()
            .withKP(gains.get(0))
            .withKI(gains.get(1))
            .withKD(gains.get(2))
            .withKS(gains.get(3))
            .withKV(gains.get(4))
            .withKA(gains.get(5))
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
            
        for (TalonFX motor : motors) {
            motor.getConfigurator().apply(newGains);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true; // 立即结束，只执行一次
    }
}
