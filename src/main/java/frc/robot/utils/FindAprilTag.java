package frc.robot.utils;

import frc.robot.Constants;
import frc.robot.Library.team19725.Point2D;
import frc.robot.Library.team19725.Point3D;
import frc.robot.utils.Models.AprilTagCoordinate;
import frc.robot.utils.Models.CandidateTagInfo;

public class FindAprilTag {
    public static double TorrentTolerance=Math.PI * 2 / 3; // Radians

    public static double[] LimeLightAngles = {0,Math.PI/2,-Math.PI/2}; // radians
    public static double DisTolerance=5.0; // meters

    public static double AngleTolerance=60.0; // degrees
    public static double AngleToleranceRad=AngleTolerance*Math.PI/180.0; // radians
    
    public static double kDis = 0.59; // 权重系数，距离越近权重越大
    public static double kAngle = 0.4; // 权重系数，角度
    public static double kTurn = 0.01; // 权重系数，转角越小权重越大

    public static double kTurnOnly = 10.0; // 转向优先时的权重系数
    private static double normalizeAngle(double ang) {
        while (ang <= -Math.PI) ang += 2.0 * Math.PI;
        while (ang > Math.PI) ang -= 2.0 * Math.PI;
        return ang;
    }

    private static double absAngleDiff(double a, double b) {
        return Math.abs(normalizeAngle(a - b));
    }

    
    private static boolean isTorrentShootable(double TrgetRobotAngle ,double DesiredTorrentAngle) {
        double diff = absAngleDiff(TrgetRobotAngle, DesiredTorrentAngle);
        return diff <= TorrentTolerance;
    }
    /*
     * 计算机器人当前位置和朝向下，最合适的AprilTag目标朝向
     * @param robotPos 机器人的3D位置
     * @param robotHeading 机器人的朝向（弧度）
     * @param DesiredTorrentAngle 期望的炮台朝向（弧度）
     * @param turnOnly 是否以机器转向最小优先
     * 如没有符合条件的tag，返回NaN
     */
    public static double getTargetHeading(Point3D robotPos, double robotHeading, double DesiredTorrentAngle, boolean turnOnly) {
        int candidateCount = 0;
        CandidateTagInfo[] candidatesInfo = new CandidateTagInfo[Constants.AprilTagCoordinates.length];
        if (Constants.AprilTagCoordinates == null) {
            return Double.NaN;
        }

        for (AprilTagCoordinate tag : Constants.AprilTagCoordinates) {
            
            //对于该Tag，最终的机器朝向
            double targetHeading = 0;
            // 过滤掉看不到的标签
            if (!Constants.usableAprilTagIDs.contains(tag.id)) {
                continue;
            }

            // 计算机器人到标签的距离,大于阈值则跳过
            double distance = Point3D.distance(robotPos, tag.position);
            if (distance > DisTolerance) {
                continue;
            }

            // 计算机器人朝向与标签法线方向的夹角,大于阈值则跳过
            Point3D TagToRobotVector = new Point3D(
                robotPos.getX() - tag.position.getX(),
                robotPos.getY() - tag.position.getY(),
                robotPos.getZ() - tag.position.getZ()
            );

            double AngleToRobot = Point3D.angleBetween(tag.nVector, TagToRobotVector);
            if (AngleToRobot > AngleToleranceRad) {
                continue;
            }

            Point3D RobotToTagVector = new Point3D(
                -TagToRobotVector.getX(),
                -TagToRobotVector.getY(),
                -TagToRobotVector.getZ()
            );

            // 将 3D 向量投影/转换到平面 2D（依赖你库的实现）
            Point2D targetHeadingVector = Point3D.toPoint2D(RobotToTagVector, new Point3D(0, 0, 1), new Point3D(0, 0, 0));
            double thisTargetHeading = Point2D.angleBetween(targetHeadingVector, new Point2D(1, 0));
            thisTargetHeading = normalizeAngle(thisTargetHeading);

            // 选择最近的 limelight 并计算需要的调整角度（归一化）
            int bestIdx = 0;
            double bestAdjust = Double.POSITIVE_INFINITY;
            for (int i = 0; i < LimeLightAngles.length; i++) {
                double thisLimeLightHeading = normalizeAngle(robotHeading + LimeLightAngles[i]);
                double adjust = absAngleDiff(thisLimeLightHeading, thisTargetHeading);
                double tmpTargetHeading = normalizeAngle(thisTargetHeading - LimeLightAngles[i]);
                if(!isTorrentShootable(tmpTargetHeading, DesiredTorrentAngle)){
                    continue;
                }
                if (adjust < bestAdjust) {
                    bestAdjust = adjust;
                    bestIdx = i;
                }
            }

            // 目标朝向（使当前最近的 limelight 指向目标）
            targetHeading = normalizeAngle(thisTargetHeading - LimeLightAngles[bestIdx]);
            
			double finalAdjustAngle = absAngleDiff(normalizeAngle(robotHeading), targetHeading);
            //TODO 计算死区

            // 满足条件的标签加入候选列表
            candidatesInfo[candidateCount] = new CandidateTagInfo(tag, distance, AngleToRobot, targetHeading, finalAdjustAngle);
            candidateCount++;
        }
        // 没有合适的标签
        if (candidateCount == 0) {
            return Double.NaN;
        }


        double thisKturn = turnOnly ? kTurnOnly : kTurn;
        // 如果只考虑转向最小，则调整权重系数

        // 选择最佳标签
        double bestScore = Double.NEGATIVE_INFINITY;
        double finalTargetHeading = 0;
        for (int i = 0; i < candidateCount; i++) {
            CandidateTagInfo candidate = candidatesInfo[i];
            // 计算评分，距离越近、角度越小评分越高
            double scoreDis = Math.abs(DisTolerance - candidate.distance) / DisTolerance;
            double scoreAngle = Math.abs(AngleToleranceRad - Math.abs(candidate.angleToRobot)) / AngleToleranceRad;
            double scoreTurn = Math.abs(AngleToleranceRad - candidate.turnRadian) / AngleToleranceRad; // 使用同一基准，越小转角越好
            double score = kDis * scoreDis + kAngle * scoreAngle + thisKturn * scoreTurn;
            if (score > bestScore) {
                bestScore = score;
                finalTargetHeading = candidate.targetHeading;
            }
        }
        return finalTargetHeading;

    }
}
