package frc.robot.utils;

import frc.robot.Constants;

import frc.robot.Library.team19725.Point2D;
import frc.robot.Library.team19725.Point3D;
import frc.robot.utils.Models.AprilTagCoordinate;
import frc.robot.utils.Models.CandidateTagInfo;

public class FindAprilTag {
    public static double DisTorlence=1.0; // meters

    public static double AngleTorlence=15.0; // degrees
    public static double AngleTorlenceRad=AngleTorlence*Math.PI/180.0; // radians

    public static double kDis = 0.5; // 权重系数，距离越近权重越大
    public static double kAngle = 0.4; // 权重系数，角度
    public static double kTurn = 0.1; // 权重系数，转角越小权重越大
    public static double getTargetHeading(Point3D robotPos, Point3D robotHeading){
        double targetHeading = 0;
        CandidateTagInfo[] candidates = new CandidateTagInfo[100];
        int candidateCount = 0;
        for (AprilTagCoordinate tag : Constants.AprilTagCoordinates) {
            // 过滤掉看不到的标签
            if (!Constants.usableAprilTagIDs.contains(tag.id)) {
                continue;
            }

            // 计算机器人到标签的距离,大于阈值则跳过
            double distance = Point3D.distance(
                new Point3D(robotPos.getX(), robotPos.getY(), robotPos.getZ()),
                tag.position
            );
            if (distance > DisTorlence) {
                continue;
            }

            // 计算机器人朝向与标签法线方向的夹角,大于阈值则跳过
            Point3D TagToRobotVector = new Point3D(
                robotPos.getX() - tag.position.getX(),
                robotPos.getY() - tag.position.getY(),
                robotPos.getZ() - tag.position.getZ()
            );
            Point3D RobotToTagVector = new Point3D(
                -TagToRobotVector.getX(),
                -TagToRobotVector.getY(),
                -TagToRobotVector.getZ()
            );
            //todo 到底有几个摄像头？？？方向？？？
            double AngleToRobot = Point3D.angleBetween(
                tag.nVector,
                TagToRobotVector
            );
            if (AngleToRobot > AngleTorlenceRad) {
                continue;
            }
            //todo 计算机器人需要旋转的角度
            //todo 计算死区

            // 满足条件的标签加入候选列表
            candidates[candidateCount] = new CandidateTagInfo(tag, distance, AngleToRobot, 0);
            candidateCount++;
        }


        // 选择最佳标签
        double bestScore = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < candidateCount; i++) {
            CandidateTagInfo candidate = candidates[i];
            // 计算评分，距离越近、角度越小评分越高
            double score = kDis * (DisTorlence - candidate.distance) / DisTorlence
                         + kAngle * (AngleTorlenceRad - candidate.angleToRobot) / AngleTorlenceRad;
            if (score > bestScore) {
                bestScore = score;
                //todo 添加计算最佳标签对应的目标朝向
            }
        }
        return targetHeading;

    }
}
