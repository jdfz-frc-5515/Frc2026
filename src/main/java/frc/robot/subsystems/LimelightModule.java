package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimelightModule {
    private static final String[] limelightNames = new String[] {
            Constants.LIME_LIGHT_ARPIL_TAG_NAME_FRONT, 
            Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT,
            Constants.LIME_LIGHT_ARPIL_TAG_NAME_LEFT,
    };
    private static final double MAX_LL_LATENCY = 100; // 100 ms, this is the maximum latency we accept from the
                                                      // limelight

    private static final boolean m_isSmartMode = true;
    public static long LastSeenAPTime = System.currentTimeMillis();
    public static void update(CommandSwerveDrivetrain swerve) {
        ChassisSpeeds chassisSpeeds = swerve.getSpeeds();

        if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 4 * Math.PI
                || Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond) > 2) {
            // 旋转速度过快或者移动速度过快都忽略
            return; 
        }

        var pigeon = swerve.getPigeon2();
        double zeroOdoDegree = swerve.getZeroOdoDegree();

        double yaw = pigeon.getYaw().getValueAsDouble() % 360;
        double pitch = pigeon.getPitch().getValueAsDouble() % 360; // 或者是 getRoll()，取决于安装方向
        double roll = pigeon.getRoll().getValueAsDouble() % 360;

        // String selectedLimelight = null;
        LimelightHelpers.PoseEstimate bestMt2 = null;
        for (int i = 0; i < limelightNames.length; ++i) {
            String llName = limelightNames[i];
            LimelightHelpers.SetRobotOrientation(llName, yaw - zeroOdoDegree, 0, pitch, 0, roll, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            if (mt2 == null) {
                continue;
            }
            if (mt2.tagCount > 0
                    && mt2.avgTagDist < 4
                    && mt2.latency < MAX_LL_LATENCY // 抛弃高延时
            ) {
                LastSeenAPTime = System.currentTimeMillis();
                if (m_isSmartMode) {
                    if (bestMt2 == null) {
                        bestMt2 = mt2;
                        // selectedLimelight = llName;
                    } else {
                        if (mt2.tagCount > bestMt2.tagCount) {
                            bestMt2 = mt2;
                        } else if (mt2.tagCount == bestMt2.tagCount && mt2.avgTagDist < bestMt2.avgTagDist) {
                            bestMt2 = mt2;
                            // selectedLimelight = llName;
                        }
                    }
                } else {
                    updateOdometry(swerve, mt2);
                }
            }
        }

        if (m_isSmartMode && bestMt2 != null) {
            updateOdometry(swerve, bestMt2);
        }
    }

    private static Matrix<N3, N1> getEstimationStdDevs(LimelightHelpers.PoseEstimate mt2) {
        // 基础标准差系数（根据你们相机的分辨率调整，默认可设为 0.1 或 0.2）
        double kPos = 0.1;

        // 如果看到多个标签，误差大幅下降
        double trustFactor = mt2.tagCount >= 2 ? 0.5 : 1.0;

        // 计算位置标准差：误差随距离平方增长
        double xyStdDev = kPos * Math.pow(mt2.avgTagDist, 2) * trustFactor;

        // 关键：动态旋转修正！
        // 如果误差较小且距离较近，我们给出一个可以接受的旋转标准差（例如 0.8 弧度）
        // 这样当陀螺仪在坡道漂移时，视觉可以慢慢把它拉回来
        double rotStdDev = (mt2.tagCount >= 2 && mt2.avgTagDist < 3.0) ? 0.8 : 999999.0;

        return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
    }

    // 如果LL看到目标，则返回到目标的距离
    // 参数minDist表示当前看到的目标距离大于这个值，则忽略此目标，负数则无效
    private static void updateOdometry(CommandSwerveDrivetrain swerve, LimelightHelpers.PoseEstimate mt2) {
        double captureTime2 = Utils.fpgaToCurrentTime(mt2.timestampSeconds);

        swerve.addVisionMeasurement(mt2.pose,
                captureTime2,
                getEstimationStdDevs(mt2));
    }

    /**
     * 寻找当前最可信的视觉旋转角度
     * 触发条件：tagCount >= 2 且 距离较近
     */
    public static Optional<Rotation2d> getTrustedVisionRotation() {
        for (String name : limelightNames) {
            // 使用 MT1 获取 BotPose，因为我们只需要它的 Rotation 独立观测值
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

            if (mt1 != null && mt1.tagCount >= 2 && mt1.avgTagDist < 3.0) {
                // 只有看到2个以上标签且距离近时，才认为这个角度是“真理”
                return Optional.of(mt1.pose.getRotation());
            }
        }
        return Optional.empty();
    }
}
