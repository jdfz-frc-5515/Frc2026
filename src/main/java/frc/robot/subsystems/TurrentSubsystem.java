package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Library.team1706.LinearInterpolationTable;
import frc.robot.commands.FeedingCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.TurnTurrentCmd;
import frc.robot.utils.MiscUtils;


public class TurrentSubsystem extends SubsystemBase {
    public static class TurrentConst {
        public static Pose2d turrentOffset = new Pose2d(0.1875-0.0127, 0.1603, Rotation2d.fromDegrees(0));
        public static double minAngle = -120;
        public static double maxAngle = 75;
        public static double kTurretDegreeForOneRotation = 14.48275862069;
        public static final double kAtTargetThreshold = 1.0; 
        
        // --- 新增：运动参数控制 ---
        public static final double kManualCruiseVelocity = 40.0; // 手动时的低速限制（圈/秒）
        public static final double kAimingCruiseVelocity = 40.0; // 自动瞄准时的快速限制（圈/秒）

        private static final double r_wheel = 0.05;      // 射击飞轮半径， 米
        
        // $\eta$ η(效率系数): 这是一个关键变量，通常在 0.5 到 0.8 之间。
        // 单轮结构（带有固定盖板/Hood）：理论上，球的中心速度只有轮缘速度的一半（因为球的一侧在动，另一侧静止在盖板上）。
        // 此时 $\eta \approx 0.5$。
        private static final double eta = 0.5;
        // 常数 C 用于换算射击飞轮的RPM和射击小球初速度
        public static final double C = (2 * Math.PI * TurrentConst.r_wheel * TurrentConst.eta);
    }

    private Trigger m_shootTrigger;
    private Trigger m_turnLeftTrigger;
    private Trigger m_turnRightTrigger;

    private ShooterEx m_shooter = new ShooterEx();
    private FeedingModule m_feeding = new FeedingModule();
    private final TalonFX m_motor = new TalonFX(Constants.TurrentMotor.motorID, new CANBus(Constants.TurrentMotor.canBusName));
    
    // 使用 MotionMagic 代替普通的 PositionVoltage，以实现平滑的低速控制
    private final MotionMagicVoltage mMMReq = new MotionMagicVoltage(0);

    private double m_targetAngle = 0; 
    private boolean m_isStartAiming = false;
    
    private CommandSwerveDrivetrain m_drivetrain;

    private double m_curMaxSpeed = 0;

    private Translation2d m_shootTarget = Constants.ShooterConstants.targetHub;
    private Pose2d m_shooterAimDir = new Pose2d(
        TurrentConst.turrentOffset.getTranslation(),
        TurrentConst.turrentOffset.getRotation()
    );
    public void setShootTrigger(Trigger trigger) throws  Exception {
        if (m_shootTrigger != null) {
            throw new Exception("setShootTrigger is called more than once!");
        }
        m_shootTrigger = trigger;
        m_shootTrigger.whileTrue(new ParallelCommandGroup(
            
            new ShooterCmd(this),
            new SequentialCommandGroup(
                new WaitCommand(0.3),
                new FeedingCmd(this)      
            )
        ));
    }

    public void setTurnLeftTrigger(Trigger trigger) throws Exception {
        if (m_turnLeftTrigger != null) {
            throw new Exception("setTurnLeftTrigger is called more than once!");
        }
        m_turnLeftTrigger = trigger;
        m_turnLeftTrigger.whileTrue(new TurnTurrentCmd(this, false));
    }

    public void setTurnRightTrigger(Trigger trigger) throws Exception {
        if (m_turnRightTrigger != null) {
            throw new Exception("setTurnRightTrigger is called more than once!");
        }
        m_turnRightTrigger = trigger;
        m_turnRightTrigger.whileTrue(new TurnTurrentCmd(this, true));
    }

    public TurrentSubsystem() {
        setupKFactorMap();
        init();
    }

    public void setDriver(CommandSwerveDrivetrain d) {
        m_drivetrain = d;
    }

    public void zeroCC() {
        m_motor.setPosition(0);
    }
    
    public void init() {
        m_shooter.init();
        m_feeding.init();
        m_motor.getConfigurator().apply(getMotorConfiguration());
        zeroCC();
    }

    @Override
    public void periodic() {
        update();
        SmartDashboard.putNumber("Turret/Current Angle", getCurrentAngle());
        SmartDashboard.putNumber("Turret/Target Angle", m_targetAngle);
        SmartDashboard.putNumber("Turret/Position", m_motor.getPosition().getValueAsDouble());
    }

    public double getCurrentAngle() {
        return -m_motor.getPosition().getValueAsDouble() * TurrentConst.kTurretDegreeForOneRotation;
    }

    protected void setSpeed(Double spd) {
        if (MiscUtils.compareDouble(spd, m_curMaxSpeed)) {
            return;
        }
        var cfg = getMotorConfiguration();
        cfg.MotionMagic.MotionMagicCruiseVelocity = spd;
        m_motor.getConfigurator().apply(cfg.MotionMagic);
        m_curMaxSpeed = spd;
    }
    /**
     * 核心控制逻辑：完全统一
     */
    public void update() {
        this.updateShooting();
        this.updateFeeding();
        // 计算目标位置（圈数）
        double motorRotationTarget = -m_targetAngle / TurrentConst.kTurretDegreeForOneRotation;
        // 始终使用 MotionMagic 运行，它会根据配置的 CruiseVelocity 自动控制速度
        m_motor.setControl(mMMReq.withPosition(motorRotationTarget));
    }

    // --- 统一后的控制接口 ---

    public void turnLeft() {
        // 设置目标为最小角度限制，MotionMagic 会负责以设定的低速转过去
        setSpeed(TurrentConst.kManualCruiseVelocity);
        // setTargetAngle(TurrentConst.minAngle);
        setTargetAngle(0);
    }

    public void turnRight() {
        // 设置目标为最大角度限制
        setSpeed(TurrentConst.kManualCruiseVelocity);
        // setTargetAngle(TurrentConst.maxAngle);
        setTargetAngle(-90);
    }

    public void stopTurn() {
        // 停止旋转：将目标设定为当前所在位置，让 PID 把它“锁”在那里
        // setTargetAngle(getCurrentAngle());
    }

    public void setTargetAngle(double angle) {
        m_targetAngle = normalizeAngle(angle, TurrentConst.minAngle, TurrentConst.maxAngle);
    }

    private double normalizeAngle(double angle, double min, double max) {
                // 确保 min <= max (根据题目描述通常满足，但做个防御)
        if (min > max) {
            // 如果配置反了，这里可以选择交换或报错，视业务而定。
            // 假设输入总是合法的 min <= max
            double temp = min;
            min = max;
            max = temp;
        }

        double range = max - min;

        // --- 核心逻辑：寻找同余解 ---
        // 目标：找到整数 k，使得 min <= angle + k * 360 <= max
        // 变换不等式：
        // k * 360 >= min - angle  =>  k >= (min - angle) / 360
        // k * 360 <= max - angle  =>  k <= (max - angle) / 360
        
        double lowerK = (min - angle) / 360.0;
        double upperK = (max - angle) / 360.0;
        
        // 计算最小的可行整数 k
        long k = (long) Math.ceil(lowerK);
        
        // 检查这个 k 是否也在上限范围内
        // 如果 range >= 360，lowerK 和 upperK 的差值 >= 1，必然存在整数 k
        if (k <= Math.floor(upperK)) {
            return angle + k * 360.0;
        }

        // --- 若无同余解：钳位到最近边界 ---
        // 计算 angle 到 min 和 max 的环形最短距离
        double distToMin = getShortestArcDistance(angle, min);
        double distToMax = getShortestArcDistance(angle, max);
        
        if (distToMin < distToMax) {
            return min;
        } else {
            return max;
        }
    }
    /**
     * 计算两个角度在圆环上的最短弧长距离 (0 ~ 180]
     */
    private static double getShortestArcDistance(double a1, double a2) {
        double diff = (a1 - a2) % 360.0;
        // 将 diff 规范化到 (-180, 180]
        if (diff <= -180) diff += 360;
        if (diff > 180) diff -= 360;
        return Math.abs(diff);
    }

    public boolean isAtTarget() {
        return Math.abs(getCurrentAngle() - m_targetAngle) < TurrentConst.kAtTargetThreshold;
    }


    private final InterpolatingDoubleTreeMap m_kFactorMap = new InterpolatingDoubleTreeMap();

    // TODO: 要测出K值
    private void setupKFactorMap() {
        // 数据点格式：m_kFactorMap.put(距离_米, 对应的K系数);
        // 这里的 K 约等于球的飞行时间 (ToF)
        // 注意：随着距离增加，K 增加的速度通常会快于距离增加的速度（因为空气阻力减速）
        m_kFactorMap.put(1.0, 0.12); // 1米处，K=0.12
        m_kFactorMap.put(2.5, 0.28); // 2.5米处
        m_kFactorMap.put(4.0, 0.45); // 4米处
        m_kFactorMap.put(6.0, 0.70); // 6米处
    }
    // 新增一个独立函数来处理偏移向量的计算
    private Translation2d calculateCompensationVector(Translation2d turretFieldVelocity, double distanceToTarget) {
        // 基础做法：常数 K
        // double kCompensationFactor = 0.2; 
        
        // 进阶做法：非线性查表 (根据距离动态获取 K 值)
        double kCompensationFactor = getKFactorFromDistance(distanceToTarget);
        
        return turretFieldVelocity.times(kCompensationFactor);
    }

    // 供未来实现非线性插值预留的方法
    private double getKFactorFromDistance(double distance) {
        return m_kFactorMap.get(distance); 
    }

    private Translation2d getShootTargetPosWithShift() {
        Translation2d originalTargetPos = ShooterConstants.targetHub;
        
        // 获取机器人的当前状态
        ChassisSpeeds chassisSpeeds = m_drivetrain.getSpeeds(); // 假设这是机器人坐标系(Robot-Centric)下的速度
        double vx = chassisSpeeds.vxMetersPerSecond;            // 机器人向前的速度
        double vy = chassisSpeeds.vyMetersPerSecond;            // 机器人向左的速度
        double vOmega = chassisSpeeds.omegaRadiansPerSecond;    // 机器人的自转角速度 (逆时针为正)
        
        // 获取机器人当前在场地中的朝向
        Rotation2d robotHeading = m_drivetrain.getPose().getRotation();

        // ---------------------------------------------------------
        // 步骤 1: 计算炮台在机器人坐标系下的局部真实速度
        // ---------------------------------------------------------
        // 因为炮台不在底盘正中心，底盘自转(vOmega)会给炮台带来额外的切向线速度。
        // 切向速度公式: v_tangential = omega x r
        double turretRx = TurrentConst.turrentOffset.getX();
        double turretRy = TurrentConst.turrentOffset.getY();
        
        // 叠加平移速度和旋转产生的切向速度
        double turretLocalVx = vx - vOmega * turretRy;
        double turretLocalVy = vy + vOmega * turretRx;
        
        // ---------------------------------------------------------
        // 步骤 2: 将局部速度转换为场地坐标系(Field-Centric)下的向量
        // ---------------------------------------------------------
        // 旋转这个速度向量，让它与场地坐标系对齐
        Translation2d turretFieldVelocity = new Translation2d(turretLocalVx, turretLocalVy).rotateBy(robotHeading);
        
        // ---------------------------------------------------------
        // 步骤 3: 做差值（线性或者非线性），并反向移动目标点
        // ---------------------------------------------------------
        // 1. 先计算出当前未补偿时，炮台到目标的直线距离
        double currentDistance = m_drivetrain.getPose().getTranslation().getDistance(originalTargetPos);

        // 2. 调用封装好的非线性补偿函数
        Translation2d shiftVector = calculateCompensationVector(turretFieldVelocity, currentDistance);

        m_shootTarget = originalTargetPos.minus(shiftVector);
        return m_shootTarget;
    }

    private void startAndUpdateAim() {
        m_isStartAiming = true;
        _updateAim();
    }

    private void stopAim() {
        m_isStartAiming = false;
    }



    public Translation2d getShootTarget() {
        return m_shootTarget;
    }

    public Pose2d getShooterAimDir() {
        return m_shooterAimDir;
    }

    // 下划线的update函数不在this.update()中调用，它被间接调用
    private void _updateAim() {
        if (m_isStartAiming) {
            double angle = calcTurrentAngle(m_drivetrain.getPose(), getShootTargetPosWithShift());
            m_shooterAimDir = new Pose2d(m_shooterAimDir.getTranslation(), Rotation2d.fromDegrees(angle));
            setSpeed(TurrentConst.kAimingCruiseVelocity);
            setTargetAngle(angle);
        }
        else {
            stopTurn();
        }
    }

    public void reset() {
        setTargetAngle(0);
    }

    private TalonFXConfiguration getMotorConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // 既然是精准定位，建议开启 Brake 模式

        // 软件限位
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = -TurrentConst.minAngle / TurrentConst.kTurretDegreeForOneRotation;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -TurrentConst.maxAngle / TurrentConst.kTurretDegreeForOneRotation;


        config.Slot0.kP = Constants.TurrentMotor.KP; // 必须有，决定力量
        config.Slot0.kI = Constants.TurrentMotor.KI; // 通常为 0
        config.Slot0.kD = Constants.TurrentMotor.KD; // 必须有，消除到达终点前的微小震荡

        // 强烈建议添加前馈参数（如果有的话）
        config.Slot0.kS = Constants.TurrentMotor.KS; // 克服静摩擦
        config.Slot0.kV = Constants.TurrentMotor.KV; // 速度前馈，让闭环更轻松
        config.Slot0.kA = Constants.TurrentMotor.KA;

        // --- 核心：Motion Magic 配置 ---
        // 通过这个配置来实现你要求的“低速”
        config.MotionMagic.MotionMagicCruiseVelocity = TurrentConst.kManualCruiseVelocity; 
        config.MotionMagic.MotionMagicAcceleration = 120.0; // 加速度，决定起步有多“肉”
        config.MotionMagic.MotionMagicJerk = .0;       // 让运动更丝滑



        // 在你的 getMotorConfiguration 方法中
        // config.Slot0.kP = Constants.TurrentMotor.KP; // 必须有，决定力量
        // config.Slot0.kI = Constants.TurrentMotor.KI; // 通常为 0
        // config.Slot0.kD = Constants.TurrentMotor.KD; // 必须有，消除到达终点前的微小震荡

        // // 强烈建议添加前馈参数（如果有的话）
        // config.Slot0.kS = Constants.TurrentMotor.KS; // 克服静摩擦
        // config.Slot0.kV = Constants.TurrentMotor.KV; // 速度前馈，让闭环更轻松

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
        // Rotation2d turretBaseAngleWorld = turretWorldPose.getRotation();
        Rotation2d turretBaseAngleWorld = m_drivetrain.getPose().getRotation();

        // 5. 计算相对旋转角度：目标角度 - 基座角度 [4]
        // 使用 Rotation2d 的 minus 方法可以自动处理角度跨越 180/-180 度的问题
        Rotation2d relativeRotation = targetAngleWorld.minus(turretBaseAngleWorld);
        double degrees = Math.toDegrees(MathUtil.angleModulus(relativeRotation.getRadians()));
        // 6. 角度归一化处理
        // 确保计算出的角度在 -180 到 180 度之间 angleModulus已完成
        // while (degrees <= -180) degrees += 360;
        // while (degrees > 180) degrees -= 360;

        // 7. 应用炮台物理旋转限制 (Clamping)
        // 炮台由于机械结构（线缆等）限制，不能在 minAngle 和 maxAngle 之外工作
        if (degrees < TurrentConst.minAngle) {
            return TurrentConst.minAngle;
        } else if (degrees > TurrentConst.maxAngle) {
            return TurrentConst.maxAngle;
        }

        return degrees;
    }

    /////////////////////////////////////////////////////////
    /// Feeding
    /////////////////////////////////////////////////////////
    
    public void reverseFeed() {
        m_feeding.reverseFeedMotor(2);      // 圈数待调
    }

    /// 先启动shooter再path再feed
    public void startFeeding() {
        m_feeding.startPathMotor();
        m_feeding.startFeedMotor();
    }

    public void stopFeeding() {
        m_feeding.stopFeedMotor();
        m_feeding.stopPathMotor();
    }

    private void updateFeeding() {
        m_feeding.update();
    }
    /////////////////////////////////////////////////////////
    /// Shooting
    /////////////////////////////////////////////////////////
    private boolean m_isShooting = false;
    public void startShooting() {
        m_isShooting = true;
    }

    public void stopShooting() {
        m_isShooting = false;
        m_shooter.stopShooting();
    }


    /**
     * 计算机器人相对于目标的径向速度 (vRadial)
     * @param robotPose 当前机器人的场地位姿
     * @param targetPos 目标的场地坐标 (通常是桶的中心)
     * @return 径向速度 (m/s)，靠近目标为正，远离为负
    */
    private double calculateRadialVelocity(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds, Translation2d targetPos) {
        // 1. 获取机器人在场地坐标系下的速度向量 (Field-relative velocity)
        // 注意：m_drivetrain.getSpeeds() 通常返回的是机器人坐标系速度
        // ChassisSpeeds robotRelativeSpeeds = m_drivetrain.getSpeeds();
        
        // 将机器人系速度转换为场地系速度向量
        Translation2d robotFieldVelocity = new Translation2d(
            robotRelativeSpeeds.vxMetersPerSecond,
            robotRelativeSpeeds.vyMetersPerSecond
        ).rotateBy(robotPose.getRotation());

        // 2. 计算从机器人指向目标的单位向量 (Direction Unit Vector)
        Translation2d robotToTargetVector = targetPos.minus(robotPose.getTranslation());
        
        // 距离太近时直接返回0，防止除以0导致的计算错误
        if (robotToTargetVector.getNorm() < 0.1) {
            return 0.0;
        }
        
        // 获取单位向量（长度为1的方向向量）
        Translation2d unitDirectionVector = robotToTargetVector.getNorm() > 0 
            ? robotToTargetVector.div(robotToTargetVector.getNorm()) 
            : new Translation2d();

        // 3. 计算点积：速度向量在方向向量上的投影
        // vRadial = Velocity_vector · Unit_Direction_vector
        // 公式：vx1*vx2 + vy1*vy2
        double vRadial = (robotFieldVelocity.getX() * unitDirectionVector.getX()) + 
                        (robotFieldVelocity.getY() * unitDirectionVector.getY());

        return vRadial;
    }
    private final LinearInterpolationTable m_rpsTable = Constants.ShooterConstants.kRPMTable;
    private void updateShooting() {
        if (m_isShooting) {
            // 1. 计算target位置，叠加机器本身速度带来的偏移
            startAndUpdateAim();
            // 2. 等待炮台转向到位
            if (isAtTarget()) {
                // 3. 开始射击
                m_shooter.startShooting();
            }
            else {
                m_shooter.stopShooting();
            }
        }
        else {
            stopAim();
        }
        if (m_shooter.getIsShooting()) {
            Pose2d robotPos = m_drivetrain.getPose();
            ChassisSpeeds chassisSpeeds = m_drivetrain.getSpeeds();
            double distance = this.getTurretWorldPose(robotPos)
                .getTranslation()
                .getDistance(Constants.ShooterConstants.targetHub);

            // TODO: 这个判断是否有必要？
            if (distance < 0.1) {
                distance = 0.1;
            }
            
            // 找到机器人速度在“机器人到桶”连线方向上的投影速度（$V_{radial}$）。
            // Calculate target speed(RPM) based on distance. 
            double targetSpeed = m_rpsTable.getOutput(distance);
            double staticVlaunch = TurrentConst.C * targetSpeed;
            // 3. 减去机器人的径向速度 (靠近为正),换算成RPM
            double vRadial = calculateRadialVelocity(robotPos, chassisSpeeds, Constants.ShooterConstants.targetHub);
            double adjustedVlaunch = (staticVlaunch - vRadial) / TurrentConst.C;

            double lastShooterSpeed = m_shooter.getTargetSpeed();
            // Prevent shooter oscillation
            if( Math.abs(adjustedVlaunch - lastShooterSpeed) < Constants.ShooterConstants.kPreventShooterOscilliationRPS ){
                adjustedVlaunch = lastShooterSpeed;
            }
            lastShooterSpeed = adjustedVlaunch;

            m_shooter.setTargetSpeed(adjustedVlaunch);

            // Push to SmartDashboard for debugging
            SmartDashboard.putNumber("ShootRPSManager/TargetSpeed", adjustedVlaunch);
            SmartDashboard.putNumber("ShootRPSManager/CurrentDistance", distance);        
        }
        m_shooter.update();
    }
}