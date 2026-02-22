package frc.robot.commands;

// this file is used for a long-time project in 5515, which will not be used in competitio
// thus, it will not affect any functionality of the robot and will not be triggered unless stated explicitly

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * 驱动电机测试 - 用于PID/SVA整定
 * 
 * 统一接口：所有测试类型通过速度列表传入
 * - 单电机测试：单位 rps (rotations per second)
 * - 底盘测试：单位 m/s
 * 
 * 使用方法：
 * - 阶跃测试：new DriveMotorTest(DriveMotorTest.createStepProfile(targetSpeed, durationSec))
 * - 斜坡/自定义测试：new DriveMotorTest(userDefinedProfile)
 * - 通过 Command.schedule() 执行
 */
public class DriveMotorTest extends Command {
    
    private static final int[] DEVICE_IDS = {1, 3, 5, 7};
    private static final double FREQUENCY = 50.0;
    
    private final List<Double> speedProfile;
    
    private final TalonFX[] motors;
    private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);
    
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
    private final boolean isChassisMode;
    
    private int currentIndex = 0;


    /**
     * 单电机测试
     * @param speedProfile 速度列表 (rps)，长度决定持续时间（50Hz控制）
     */
    public DriveMotorTest(List<Double> speedProfile) {
        this.speedProfile = Collections.unmodifiableList(new ArrayList<>(speedProfile));
        this.isChassisMode = false;
        this.drivetrain = null;
        
        motors = new TalonFX[DEVICE_IDS.length];
        for (int i = 0; i < DEVICE_IDS.length; i++) {
            motors[i] = new TalonFX(DEVICE_IDS[i]);
        }
    }
    
    /**
     * 底盘测试
     * @param drivetrain 底盘子系统
     * @param speedProfile 速度列表 (m/s)，长度决定持续时间（50Hz控制）
     */
    public DriveMotorTest(CommandSwerveDrivetrain drivetrain, List<Double> speedProfile) {
        this.speedProfile = Collections.unmodifiableList(new ArrayList<>(speedProfile));
        this.isChassisMode = true;
        this.drivetrain = drivetrain;
        this.motors = null;
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        currentIndex = 0;
        if (!speedProfile.isEmpty()) {
            applySpeed(speedProfile.get(0));
        }
    }
    
    @Override
    public void execute() {
        if (currentIndex < speedProfile.size()) {
            applySpeed(speedProfile.get(currentIndex));
            currentIndex++;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        applySpeed(0.0);
    }
    
    @Override
    public boolean isFinished() {
        return currentIndex >= speedProfile.size();
    }
    
    /**
     * 生成阶跃速度曲线（恒定速度）
     * @param targetSpeed 目标速度 (rps 或 m/s)
     * @param durationSec 持续时间 (s)
     * @return 速度列表
     */
    public static List<Double> createStepProfile(double targetSpeed, double durationSec) {
        int size = (int) Math.ceil(durationSec * FREQUENCY);
        List<Double> profile = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            profile.add(targetSpeed);
        }
        return profile;
    }
    
    private void applySpeed(double speed) {
        if (isChassisMode) {
            drivetrain.setControl(robotCentricRequest.withVelocityX(speed).withVelocityY(0).withRotationalRate(0));
        } else {
            for (TalonFX motor : motors) {
                motor.setControl(velocityRequest.withVelocity(speed));
            }
        }
    }
}
