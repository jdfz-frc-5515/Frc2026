package frc.robot.subsystems;

import java.util.logging.LogManager;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MessageSender;


public class TurrentSystem extends SubsystemBase{
    public static class TurrentConst {
        public static Pose2d turrentOffset = new Pose2d(0, 0, Rotation2d.fromDegrees(0));       // 炮台相对于机器人中心的偏移（包含 X/Y 偏移和初始旋转）
        public static double minAngle = -140;      // 炮台旋转的最小角度限制（度）
        public static double maxAngle = 140;       // 炮台旋转的最大角度限制（度）
    }

    private final TalonFX m_motor = new TalonFX(Constants.TurrentMotor.motorID, new CANBus(Constants.TurrentMotor.canBusName));
    // private MotionMagicVoltage m_mmv = new MotionMagicVoltage(0);
    private PositionVoltage m_mmv = new PositionVoltage(0);
    private int m_turnState = 0;    // 1正转，-1反转，0不转

    private DutyCycleOut dc = new DutyCycleOut(0);
    public TurrentSystem() {
        init();
    }

    public void init() {
        m_motor.getConfigurator().apply(getMotorConfiguration());
        m_motor.setPosition(0);
    }

    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        update();
    }

    private double getMotorPosition() {
        double pos = m_motor.getPosition().getValueAsDouble();
        return pos;
    }
    public void update() {
        double DT = 0.1;
        double curPos = getMotorPosition();
        // MessageSender.log(String.format("tuurent pos: %f", curPos));
        switch (m_turnState) {
            case 0:
                m_motor.stopMotor();
                break;
            case 1:
                // MessageSender.log("turn 1");
                m_motor.setControl(m_mmv.withPosition(-6));
                // m_motor
                // m_motor.setControl(dc.withOutput(0.05));
                break;
            case -1:
                // MessageSender.log("turn -1");
                m_motor.setControl(m_mmv.withPosition(6));
                // m_motor.setControl(dc.withOutput(-0.05));
                break;
            default:
                break;
        }
    }

    public void turnLeft() {
        m_turnState = -1;
    }

    public void turnRight() {
        m_turnState = 1;
    }

    public void stopTurn() {
        m_turnState = 0;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private TalonFXConfiguration getMotorConfiguration() {
            TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        // elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -200;
        // elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.Slot0.kP = Constants.TurrentMotor.KP;
        config.Slot0.kI = Constants.TurrentMotor.KI;
        config.Slot0.kD = Constants.TurrentMotor.KD;
        config.Slot0.kS = Constants.TurrentMotor.KS;
        config.Slot0.kV = Constants.TurrentMotor.KV;
        config.Slot0.kA = Constants.TurrentMotor.KA;

        return config;
    }

    /**
     * 计算炮台在场地坐标系下的位姿（位置 + 朝向）。
     * 将炮台相对于机器人中心的偏移（turrentOffset）应用到机器人位姿上。
     *
     * @param robotPos 机器人在场地坐标系下的位姿
     * @return 炮台在场地坐标系下的 Pose2d（位置 + 朝向）
     */
    public Pose2d getTurretWorldPose(Pose2d robotPos) {
        // 炮台在机器人坐标系下的平移量需要以机器人当前朝向旋转后再加到机器人原点
        Translation2d worldTrans = robotPos.getTranslation().plus(
            TurrentConst.turrentOffset.getTranslation().rotateBy(robotPos.getRotation())
        );
        // 炮台的朝向为机器人朝向加上炮台安装时的旋转偏移
        Rotation2d worldRot = robotPos.getRotation().plus(TurrentConst.turrentOffset.getRotation());
        return new Pose2d(worldTrans, worldRot);
    }

    /**
     * 计算炮台瞄准目标所需的相对旋转角度
     * 
     * @param robotPos  当前机器人在场地上的姿态 (World Frame)
     * @param targetPos 瞄准目标在场地上的坐标
     * @return 炮台相对于其初始位置的旋转角度（度），已应用限制
     */
    public double calcTurrentAngle(Pose2d robotPos, Translation2d targetPos) {
        // 1. 计算炮台在场地上的实际世界位姿 (位置 + 朝向)
        // 提取为独立方法 getTurretWorldPose(robotPos)
        Pose2d turretWorldPose = getTurretWorldPose(robotPos);
        Translation2d turretWorldPos = turretWorldPose.getTranslation();

        // 2. 计算从炮台位置指向目标的向量
        Translation2d vectorToTarget = targetPos.minus(turretWorldPos);

        // 3. 计算目标相对于场地坐标系的角度 (World Frame Angle)
        // 使用 Math.atan2 计算弧度并转为 Rotation2d
        Rotation2d targetAngleWorld = new Rotation2d(vectorToTarget.getX(), vectorToTarget.getY());

    // 4. 计算炮台基座（0度参考位）在场地坐标系中的当前角度
    // 使用独立方法返回的位姿中的旋转部分
    Rotation2d turretBaseAngleWorld = turretWorldPose.getRotation();

        // 5. 计算相对旋转角度：目标角度 - 基座角度 [4]
        // 使用 Rotation2d 的 minus 方法可以自动处理角度跨越 180/-180 度的问题
        Rotation2d relativeRotation = targetAngleWorld.minus(turretBaseAngleWorld);
        double degrees = relativeRotation.getDegrees();

        // 6. 角度归一化处理
        // 确保计算出的角度在 -180 到 180 度之间
        while (degrees <= -180) degrees += 360;
        while (degrees > 180) degrees -= 360;

        // 7. 应用炮台物理旋转限制 (Clamping)
        // 炮台由于机械结构（线缆等）限制，不能在 minAngle 和 maxAngle 之外工作
        if (degrees < TurrentConst.minAngle) {
            return TurrentConst.minAngle;
        } else if (degrees > TurrentConst.maxAngle) {
            return TurrentConst.maxAngle;
        }

        return degrees;
    }
}
