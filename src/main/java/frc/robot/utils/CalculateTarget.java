package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Library.team19725.Point2D;
import frc.robot.Library.team19725.Point3D;

public class CalculateTarget {
    //TODO 填入准确的场地参数
    protected static double TrenchHeight = 0.2; //传球高度
    protected static double TrenchX_Blue = 3;
    protected static double TrenchX_Red = 7;
    //X坐标范围限制，如不满足说明在联盟区域内，不应当传球
    protected static double XThreshold_Blue = 4;
    protected static double XThreshold_Red = 6;
    //受hub影响，传球有一个死区，该区域内无法传球
    protected static double hubY1 = 4;
    protected static double hubY2 = 6;
    protected static double hubYMid = 5;
    protected static Point2D Blue1 = new Point2D(XThreshold_Blue, hubY1);
    protected static Point2D Blue2 = new Point2D(XThreshold_Blue + 2, hubYMid);
    protected static Point2D Blue3 = new Point2D(XThreshold_Blue, hubY2);
    protected static Point2D Red1 = new Point2D(XThreshold_Red, hubY1);
    protected static Point2D Red2 = new Point2D(XThreshold_Red - 2, hubYMid);
    protected static Point2D Red3 = new Point2D(XThreshold_Red, hubY2);

    /**
     * 计算传球目标点
     * @param robotPos 机器人当前位置
     * @param alliance 联盟颜色
     * @return 传球目标点,如无法传球，则返回 (NaN, NaN, NaN)
     */
    public static Point3D calculatePassTarget(Point3D robotPos, Alliance alliance) {
        double robotY = robotPos.getY();
        double robotX = robotPos.getX();
        //蓝方
        if(alliance == Alliance.Blue) {
            if(robotX < XThreshold_Blue) {
                return new Point3D(Double.NaN, Double.NaN, Double.NaN); //在联盟区域内，不传球
            }
            else if(isPointInTriangle(new Point2D(robotX, robotY), Blue1, Blue2, Blue3)) {
                return new Point3D(Double.NaN, Double.NaN, Double.NaN); //在死区内，不传球
            }
            //无遮挡
            if(robotY > hubY2 || robotY < hubY1) {
                return new Point3D(TrenchX_Blue, robotY, TrenchHeight); //蓝方传球目标点
            }
            //在下半区
            else if(robotY <= hubYMid) {
                double k = (robotY - hubY1) / (robotX - XThreshold_Blue);
                double targetY = robotY - (robotX - TrenchX_Blue) * k;
                return new Point3D(TrenchX_Blue, targetY, TrenchHeight);
            }
            //在上半区
            else{
                double k = (hubY2 - robotY) / (robotX - XThreshold_Blue);
                double targetY = robotY + (robotX - TrenchX_Blue) * k;
                return new Point3D(TrenchX_Blue, targetY, TrenchHeight);
            }
        }
        //红方
        else {
            if(robotX > XThreshold_Red) {
                return new Point3D(Double.NaN, Double.NaN, Double.NaN); //在联盟区域内，不传球
            }
            else if(isPointInTriangle(new Point2D(robotX, robotY), Red1, Red2, Red3)) {
                return new Point3D(Double.NaN, Double.NaN, Double.NaN); //在死区内，不传球
            }
            //无遮挡
            if(robotY > hubY2 || robotY < hubY1) {
                return new Point3D(TrenchX_Red, robotY, TrenchHeight); //红方传球目标点
            }
            //在下半区
            else if(robotY <= hubYMid) {
                double k = (robotY - hubY1) / (XThreshold_Red - robotX);
                double targetY = robotY - (TrenchX_Red - robotX) * k;
                return new Point3D(TrenchX_Red, targetY, TrenchHeight);
            }
            //在上半区
            else{
                double k = (hubY2 - robotY) / (XThreshold_Red - robotX);
                double targetY = robotY + (TrenchX_Red - robotX) * k;
                return new Point3D(TrenchX_Red, targetY, TrenchHeight);
            }
        }
    }

    /**
     * 判断点 p 是否在由顶点 a,b,c 定义的三角形内（包含边界）
     * @param p
     * @param a
     * @param b
     * @param c
     * @return
     */
    public static boolean isPointInTriangle(Point2D p, Point2D a, Point2D b, Point2D c) {
        final double EPS = 1e-9;

        double d1 = sign(p, a, b);
        double d2 = sign(p, b, c);
        double d3 = sign(p, c, a);

        boolean hasNeg = (d1 < -EPS) || (d2 < -EPS) || (d3 < -EPS);
        boolean hasPos = (d1 > EPS)  || (d2 > EPS)  || (d3 > EPS);

        return !(hasNeg && hasPos); // 若同时存在正负则在三角形外，否者在内或在边上
    }

    /**
     *  计算有向面积的两倍（即叉积）
     * @param p1
     * @param p2
     * @param p3
     * @return
     */
    private static double sign(Point2D p1, Point2D p2, Point2D p3) {
        return (p1.getX() - p3.getX()) * (p2.getY() - p3.getY())
             - (p2.getX() - p3.getX()) * (p1.getY() - p3.getY());
    }
}
