package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class TurrentSystem {
    public static class TurrentConst {
        public static Pose2d turrentOffset = new Pose2d(0, 0, Rotation2d.fromDegrees(0));       // 炮台相对于机器人中心的偏移（包含 X/Y 偏移和初始旋转）
        public static double minAngle = -140;      // 炮台旋转的最小角度限制（度）
        public static double maxAngle = 140;       // 炮台旋转的最大角度限制（度）
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
