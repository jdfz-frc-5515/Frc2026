package frc.robot.commands;

// this file is used for a long-time project in 5515, which will not be used in competitio
// thus, it will not affect any functionality of the robot and will not be triggered unless stated explicitly

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * 为PIDSVA整定记录电机实时速度
 * 
 * 使用方法：
 * - 调用 startRecording() 开始记录
 * - 调用 stopRecording() 停止记录并获取速度数组
 */
public class DataLogger extends Command {    
    private static final int[] DEVICE_IDS = {1, 3, 5, 7};
    private static final double UPDATE_FREQUENCY = 100.0;
    private static final double SAMPLE_PERIOD_MS = 10.0;
    
    private final TalonFX[] motors;
    private final StatusSignal<AngularVelocity>[] velocitySignals;
    
    private volatile boolean isRecording = false;
    
    private final List<Double>[] velocityData; // 每个电机的速度列表
    private final Notifier notifier; // Notifier定时器用于多线程


    @SuppressWarnings("unchecked")
    public DataLogger() {
        int motorCount = DEVICE_IDS.length;
        
        motors = new TalonFX[motorCount];
        velocitySignals = new StatusSignal[motorCount];
        velocityData = new ArrayList[motorCount];
        
        for (int i = 0; i < motorCount; i++) {
            motors[i] = new TalonFX(DEVICE_IDS[i]);
            velocitySignals[i] = motors[i].getVelocity();
            velocityData[i] = new ArrayList<>();
            
            // 设置信号更新频率为100Hz
            velocitySignals[i].setUpdateFrequency(UPDATE_FREQUENCY);
        }
        
        notifier = new Notifier(this::recordSample);
    }
    
    public void startRecording() {
        if (isRecording) {
            return;
        }
        
        for (List<Double> data : velocityData) {
            data.clear();
        }
        
        isRecording = true;
        
        notifier.startPeriodic(SAMPLE_PERIOD_MS / 1000.0);
    }
    
    private void recordSample() {
        if (!isRecording) {
            return;
        }
        
        BaseStatusSignal.refreshAll(velocitySignals);
        
        // 记录四个电机的速度值 (单位为 rotations per second)
        for (int i = 0; i < velocitySignals.length; i++) {
            double velocity = velocitySignals[i].getValueAsDouble();
            velocityData[i].add(velocity);
        }
    }
    
    /**
     * 停止记录并返回速度数组
     * 
     * @return 二维数组，第一维是电机索引(示例：0-3对应deviceId 1,3,5,7)，第二维是速度数据
     *         单位为 rotations per second
     */
    public double[][] stopRecording() {
        isRecording = false;
        
        // 停止Notifier
        notifier.stop();
        
        // 将List转换为数组
        double[][] result = new double[velocityData.length][];
        for (int i = 0; i < velocityData.length; i++) {
            List<Double> data = velocityData[i];
            result[i] = new double[data.size()];
            for (int j = 0; j < data.size(); j++) {
                result[i][j] = data.get(j);
            }
        }
        
        return result;
    }
    
    /**
     * 检查是否正在记录
     * 
     * @return true if recording, false otherwise
     */
    public boolean isRecording() {
        return isRecording;
    }
    
    /**
     * 获取当前记录的数据点数
     * 
     * @return 每个电机的数据点数
     */
    public int getSampleCount() {
        if (velocityData.length > 0) {
            return velocityData[0].size();
        }
        return 0;
    }
    
    /**
     * 获取Device ID数组 (1, 3, 5, 7)
     * 
     * @return Device ID数组
     */
    public static int[] getDeviceIds() {
        return DEVICE_IDS.clone();
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
    }
    
    @Override
    public void end(boolean interrupted) {
    }
}
