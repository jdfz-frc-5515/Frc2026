package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
// import com.thethriftybot.Conversion;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import frc.robot.Library.MUtils.SegmentOnTheField;
import frc.robot.Library.MUtils.SegmentOnTheField;
import frc.robot.utils.Models.AprilTagCoordinate;
import frc.robot.Library.team1706.LinearInterpolationTable;

import java.awt.geom.Point2D;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Constants {
    public static Alliance alliance = Alliance.Blue;
    public static String auto_path = "Blue_UT_A_SHOOT";
    public static Pose2d auto_start_pos = 
        new Pose2d(new Translation2d(4.435, 7.546), 
            Rotation2d.fromDegrees(alliance == Alliance.Red ? 0 : 180));

    public static String LIME_LIGHT_ARPIL_TAG_NAME_RIGHT = "limelight-right";
    public static String LIME_LIGHT_ARPIL_TAG_NAME_LEFT = "limelight-left";
    public static String LIME_LIGHT_ARPIL_TAG_NAME_FRONT = "limelight-front";

    public static class GlobalConstants {
        public final static float INF = (float) Math.pow(10, 6); // this was defined for the 1690 lib
    }

    public static class DriveConstants {
        public final static double kInnerDeadband = 0.004;
        public final static double kOuterDeadband = 0.98; // these were defined for the 1706 lib;
        public static final int IntakeButton = 0;
    }

    public static final class AutoConstants {
        // Path Following
        public static final double followPathTranslationkP = 14.;  
        public static final double followPathTranslationkI = 2.;  
        public static final double followPathTranslationkD = 0.;  

        public static final double followPathRotationkP = 10.;  
        public static final double followPathRotationkI = 2.;  
        public static final double followPathRotationkD = 0.;  

        public static final PathConstraints generatedPathCommandConstraints = new PathConstraints(  
                3.,
                3.,
                Units.degreesToRadians(540.),
                Units.degreesToRadians(720.));

        // Move To Pose
        public static final double moveToPoseTranslationkP = 3;  
        public static final double moveToPoseTranslationkI = 0.;  
        public static final double moveToPoseTranslationkD = 0.0;  

        public static final double moveToPoseRotationkP = 5;  
        public static final double moveToPoseRotationkI = 2.;  
        public static final double moveToPoseRotationkD = 0.;  

        public static final double moveToPoseRotationToleranceRadians = Units.degreesToRadians(3.);  
        public static final double moveToPoseTranslationToleranceMeters = 0.02;  

        public static final double maxMoveToSpeed = 3;  
        public static final double maxMoveToAngularVelocity = Units.degreesToRadians(200.);  

    }

    public static void initializeConstants() {
    }

    public static final class FieldConstants {

        public static final Translation2d FieldCenter = new Translation2d(17.548225 / 2, 8.0518 / 2.);
        public static final Translation2d BlueRightStationCenterPos = new Translation2d(1.2, 1.05);
        public static final Translation2d DStationTranslationRSL = new Translation2d(-0.675, 0.42);
        public static final Rotation2d DStationRotationRSL = Rotation2d.fromRadians(0.935);

        public static Pose2d rotateAroundCenter(Pose2d pose, Translation2d centre, Rotation2d rotation) {
            return new Pose2d(pose.getTranslation().rotateAround(centre, rotation), pose.getRotation().plus(rotation));
        }

        public static final Translation2d BlueRghtStationStPos = new Translation2d(0.6, 1.31);
        public static final Translation2d BlueRghtStationEdPos = new Translation2d(1.6, 0.56);

        public static final Translation2d BlueLeftStationStPos = new Translation2d(BlueRghtStationStPos.getX(),
                FieldCenter.getY() * 2 - BlueRghtStationStPos.getY());
        public static final Translation2d BlueLeftStationEdPos = new Translation2d(BlueRghtStationEdPos.getX(),
                FieldCenter.getY() * 2 - BlueRghtStationEdPos.getY());

        public static final SegmentOnTheField BlueRghtStation = new SegmentOnTheField(BlueRghtStationStPos,
                BlueRghtStationEdPos);
        public static final SegmentOnTheField BlueLeftStation = new SegmentOnTheField(BlueLeftStationStPos,
                BlueLeftStationEdPos);

        public static final double StationDetectionArea = 0.3;

        // 换成Welded场地坐标
        public static AprilTagCoordinate[] AprilTagCoordinates = new AprilTagCoordinate[] {
            // 战壕 (Trench) - 高度 35.00" (0.89m)
            new AprilTagCoordinate(1, 11.878, 7.425, 0.889, -1.0, 0.0, 0.0, 1.41), // 180.0°
            new AprilTagCoordinate(6, 11.878, 0.644, 0.889, -1.0, 0.0, 0.0, 1.41), // 180.0°
            new AprilTagCoordinate(7, 11.953, 0.644, 0.889, 1.0, 0.0, 0.0, 1.41), // 0.0°
            new AprilTagCoordinate(12, 11.953, 7.425, 0.889, 1.0, 0.0, 0.0, 1.41), // 0.0°
            // 中枢 (Hub) - 高度 44.25" (1.12m)
            new AprilTagCoordinate(2, 11.915, 4.638, 1.124, 0.0, 1.0, 0.0, 1.54), // 90.0°
            new AprilTagCoordinate(3, 11.312, 4.390, 1.124, -1.0, 0.0, 0.0, 1.54), // 180.0°
            new AprilTagCoordinate(4, 11.312, 4.035, 1.124, -1.0, 0.0, 0.0, 1.54), // 180.0°
            new AprilTagCoordinate(5, 11.915, 3.431, 1.124, 0.0, -1.0, 0.0, 1.54), // 270.0°
            new AprilTagCoordinate(8, 12.271, 3.431, 1.124, 0.0, -1.0, 0.0, 1.54), // 270.0°
            new AprilTagCoordinate(9, 12.519, 3.679, 1.124, 1.0, 0.0, 0.0, 1.54), // 0.0°
            new AprilTagCoordinate(10, 12.519, 4.035, 1.124, 1.0, 0.0, 0.0, 1.54), // 0.0°
            new AprilTagCoordinate(11, 12.271, 4.638, 1.124, 0.0, 1.0, 0.0, 1.54), // 90.0°
            // 哨站 (Outpost) - 高度 21.75" (0.55m)
            new AprilTagCoordinate(13, 16.533, 7.403, 0.552, -1.0, 0.0, 0.0, 0.93), // 180.0°
            new AprilTagCoordinate(14, 16.533, 6.972, 0.552, -1.0, 0.0, 0.0, 0.93), // 180.0°
            // 塔墙 (Tower Wall) - 高度 21.75" (0.55m)
            new AprilTagCoordinate(15, 16.533, 4.324, 0.552, -1.0, 0.0, 0.0, 0.93), // 180.0°
            new AprilTagCoordinate(16, 16.533, 3.892, 0.552, -1.0, 0.0, 0.0, 0.93), // 180.0°
            new AprilTagCoordinate(17, 4.663, 0.644, 0.889, 1.0, 0.0, 0.0, 1.41), // 0.0°
            new AprilTagCoordinate(18, 4.626, 3.431, 1.124, 0.0, -1.0, 0.0, 1.54), // 270.0°
            new AprilTagCoordinate(19, 5.229, 3.679, 1.124, 1.0, 0.0, 0.0, 1.54), // 0.0°
            new AprilTagCoordinate(20, 5.229, 4.035, 1.124, 1.0, 0.0, 0.0, 1.54), // 0.0°
            new AprilTagCoordinate(21, 4.626, 4.638, 1.124, 0.0, 1.0, 0.0, 1.54), // 90.0°
            new AprilTagCoordinate(22, 4.663, 7.425, 0.889, 1.0, 0.0, 0.0, 1.41), // 0.0°
            new AprilTagCoordinate(23, 4.588, 7.425, 0.889, -1.0, 0.0, 0.0, 1.41), // 180.0°
            new AprilTagCoordinate(24, 4.270, 4.638, 1.124, 0.0, 1.0, 0.0, 1.54), // 90.0°
            new AprilTagCoordinate(25, 4.022, 4.390, 1.124, -1.0, 0.0, 0.0, 1.54), // 180.0°
            new AprilTagCoordinate(26, 4.022, 4.035, 1.124, -1.0, 0.0, 0.0, 1.54), // 180.0°
            new AprilTagCoordinate(27, 4.270, 3.431, 1.124, 0.0, -1.0, 0.0, 1.54), // 270.0°
            new AprilTagCoordinate(28, 4.588, 0.644, 0.889, -1.0, 0.0, 0.0, 1.41), // 180.0°
            new AprilTagCoordinate(29, 0.008, 0.666, 0.552, 1.0, 0.0, 0.0, 0.93), // 0.0°
            new AprilTagCoordinate(30, 0.008, 1.098, 0.552, 1.0, 0.0, 0.0, 0.93), // 0.0°
            new AprilTagCoordinate(31, 0.008, 3.746, 0.552, 1.0, 0.0, 0.0, 0.93), // 0.0°
            new AprilTagCoordinate(32, 0.008, 4.178, 0.552, 1.0, 0.0, 0.0, 0.93), // 0.0°

        };

        //排除掉1,3,4,6,17，19，20，22号八个看不到的Tag;排除trench上AP（只有一个）
        public static final List<Integer> usableAprilTagIDs = Arrays.asList(
                //2,5,7,8,9,10,11,12,13,14,15,16,18, 21, 23, 24, 25, 26, 27,28, 29, 30, 31, 32
                2,5,8,9,10,11,13,14,15,16,18, 21, 24, 25, 26, 27, 29, 30, 31, 32
        );

    }

    public static final class PathPlanner {
        public static final double constraintsSpeed = 1.;
        public static final double constraintsAccel = 1.;
    }

    public final class FieldInfo {
        // 常量Map定义
        public static final Map<Long, APInfo> AP_MAP;
        public static final long[] blueApIds = {
            17, 18, 19, 20, 21, 22,
        };
        public static final long[] redApIds = {
            6, 7, 8, 9, 10, 11,
        }; 

        public static double coralBranchOffset = 0.164338;
        public static double coralVerticalOffset = 0.525;
        public static double agleaVerticalOffset = 0.8;
        public static double sourceHorizontalOffset = 0.0; // 水平偏移量 +往右偏（面向source方向）
        public static double sourceVerticalOffset = -(0.857 / 2 + 0.00);  // 车子尺寸：0.857x0.857， 距离bump的距离是0.02米
        
        // 接入点信息类
        public static class APInfo {
            private final int id;
            private final double x;
            private final double y;
            private final int theta;    // 这个是机器对着它的角度，机器的角度
            private final int theta2;   // 这个是AprilTag所在边的角度
            private final int algea;    // 0和1代表Algea在上层还是下层
            private final List<Integer> sourceList;

            public APInfo(int id, double x, double y, int theta, int theta2, int algea, List<Integer> sourceList) {
                this.id = id;
                this.x = x;
                this.y = y;
                this.theta = theta;
                this.theta2 = theta2;
                this.algea = algea;
                this.sourceList = sourceList;
            }

            // Getter方法
            public int getId() { return id; }
            public double getX() { return x; }
            public double getY() { return y; }
            public int getTheta() { return theta; }
            public int getTheta2() { return theta2; }
            public int getAlgea() { return algea; }
            public List<Integer> getSourceList() { return sourceList; }
        }

        // 静态初始化常量Map
        static {
            Map<Long, APInfo> map = new HashMap<>();
            
            map.put(17l, new APInfo(17, 4.073906, 3.306318, 60, 330, 0, Arrays.asList(12)));
            map.put(18l, new APInfo(18, 3.6576, 4.0259, 0, 270, 1, Arrays.asList(12, 13)));
            map.put(19l, new APInfo(19, 4.073906, 4.745482, -60, 210, 0, Arrays.asList(13)));
            map.put(20l, new APInfo(20, 4.90474, 4.7475482, -120, 150, 1, Arrays.asList(13)));
            map.put(21l, new APInfo(21, 5.321046, 4.0259, 180, 90, 0, Arrays.asList(12, 13)));
            map.put(22l, new APInfo(22, 4.90474, 3.306318, 120, 30, 1, Arrays.asList(12)));
            map.put(6l, new APInfo(6, 13.474446, 3.306318, 120, 30, 0, Arrays.asList(1)));
            map.put(7l, new APInfo(7, 13.890498, 4.0259, 180, 90, 1, Arrays.asList(1, 2)));
            map.put(8l, new APInfo(8, 13.474446, 4.745482, -120, 150, 0, Arrays.asList(2)));
            map.put(9l, new APInfo(9, 12.643358, 4.745482, -60, 210, 1, Arrays.asList(2)));
            map.put(10l, new APInfo(10, 12.227306, 4.0259, 0, 270, 0, Arrays.asList(1, 2)));
            map.put(11l, new APInfo(11, 12.643358, 3.306318, 60, 330, 1, Arrays.asList(1)));
            
            map.put(1l, new APInfo(8, 16.7, 0.655, 126, 0, -1, Arrays.asList()));
            map.put(2l, new APInfo(9, 16.7, 7.39648, 234, 0, -1, Arrays.asList()));
            map.put(12l, new APInfo(10, 0.851, 0.655, 54, 0, -1, Arrays.asList()));
            map.put(13l, new APInfo(11, 0.851, 7.39648, 306, 0, -1, Arrays.asList()));
            


            AP_MAP = Collections.unmodifiableMap(map);
        }

        
        // 私有构造防止实例化
        private FieldInfo() {}
    }

    public static final class ShooterConstants {
        public static final int PRIMARY_CAN_ID = 10;
        public static final int FOLLOWER_CAN_ID = 11;
        public static final String canBusName = "rio";
        public static final double idleSpeed = 0.0;
        public static final double shootingSpeed = 60.0;
        public static final double KP = 0.2;
        public static final double KI = 0.002;
        public static final double KD = 0.0015;
        public static final double KS = 0.15;
        public static final double KV = 0.125;
        public static final double KA = 10.0; // velocity setpoint dont use ka
        public static final double kPreventShooterOscilliationRPS = 1.0;

        // // hard ball
        // private static final Point2D[] kRPMPoints = new Point2D.Double[] {
        //     // (distance, shooterSpeedRPS) for hard balls
        //     new Point2D.Double(-1, 40.97),
        //     new Point2D.Double(1.5, 40.97),
        //     new Point2D.Double(2, 44),
        //     new Point2D.Double(2.5, 49.14),
        //     new Point2D.Double(3, 50.68),
        //     new Point2D.Double(3.5, 54.86),
        //     new Point2D.Double(4, 54.52),
        //     new Point2D.Double(100, 62.71),
        // };
        // private static final Point2D[] kShotTimePoints = new Point2D.Double[] {
        //     // HardBall
        //     // (distance, seconds)
        //     // TODO: 要测出K值
        //     new Point2D.Double(-100.0, 0.92),
        //     new Point2D.Double(1.5, 0.92),
        //     new Point2D.Double(2, 1.045),
        //     new Point2D.Double(2.5, 1.16),
        //     new Point2D.Double(3, 1.18),
        //     new Point2D.Double(3.5, 1.39),
        //     new Point2D.Double(4, 1.54),
        //     new Point2D.Double(100.0, 1.54),
        // };

        // soft ball
        private static final Point2D[] kRPMPoints = new Point2D.Double[] {
            // (distance, shooterSpeedRPS) for soft balls
            new Point2D.Double(-100, 40.97),
            new Point2D.Double(1.5, 40.97),
            new Point2D.Double(2, 44),
            new Point2D.Double(2.5, 46),
            new Point2D.Double(3, 55.75),
            new Point2D.Double(3.5, 76.68),
            new Point2D.Double(4, 97),
            new Point2D.Double(100, 97),    
        };
        
        private static final Point2D[] kShotTimePoints = new Point2D.Double[] {
            // SoftBall
            // (distance, seconds)
            // TODO: 要测出K值
            new Point2D.Double(-100.0, 0.92),
            new Point2D.Double(1.5, 0.92),
            new Point2D.Double(2, 0.95),
            new Point2D.Double(2.5, 1.08),
            new Point2D.Double(3, 1.17),
            new Point2D.Double(3.5, 1.35),
            new Point2D.Double(4, 1.365),
            new Point2D.Double(100.0, 1.365),
        };

        public static final LinearInterpolationTable kRPMTable = new LinearInterpolationTable(kRPMPoints);
        public static final LinearInterpolationTable kShotTimeTable = new LinearInterpolationTable(kShotTimePoints);
        private static final Translation2d blueHub = new Translation2d(4.022+0.55, 4.021328);    // 0.55是hub的半径
        private static final Translation2d redHub = new Translation2d(12.519-0.55, 4.021328);
        public static final Translation2d targetHub = (alliance == Alliance.Blue) ? blueHub : redHub;
    }

    public static final class FeedingConstants {
        public static final double FEED_STATIC_VELOCITY = 0.0;
        public static final double FEED_SPIN_VELOCITY = 2.0; // rps
        public static final double PATH_STATIC_POWER = 0.0;
        public static final double PATH_SPIN_POWER = 0.3;
        public static final int FEED_MOTOR_CAN_ID = 6;
        public static final int PATH_MOTOR_CAN_ID = 7;
        public static final double KP = 0.1;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KS = 0.04;
        public static final double KV = 0.115;
        public static final double KA = 0.0;
    }

    public static final class IntakeConstants {
        public static final String canBusName = "rio";
        public static double Intake_Voltage = 8;
        public static double Extender_Voltage = 1;
        public static double Extender_Push_Voltage = 3; 
        public static double Tolerance = 0.5;
        public static final int INTAKE_CAN_ID = 19;
        public static final int EXTENDER_CAN_ID = 20;
        public static final double GEAR_RATIO = 27.0;

        //In：Extender推蓝色壳子到推不动，齿轮自然停下来为止（收最里面）；
        //out同理，放出后拉到顺时针再也拉不动为止
        public static final double IN_POS = 0.3;
        public static final double OUT_POS = 5;

        public static final double EXTENDER_KP = 1;
        public static final double EXTENDER_KI = 0.0;
        public static final double EXTENDER_KD = 0.0;
        public static final double EXTENDER_KS = 0.0;
        public static final double EXTENDER_KV = 0.0;
        public static final double EXTENDER_KA = 0.0;
        public static final double INTAKE_KP = 0;
        public static final double INTAKE_KI = 0.0;
        public static final double INTAKE_KD = 0.0;
        public static final double INTAKE_KS = 0.;
        public static final double INTAKE_KV = 0.;
        public static final double INTAKE_KA = 0.0;

        //TODO: check current limit
        public static final double INTAKE_STATOR_CURRENT_LIMIT = 50;
        public static final double INTAKE_SUPLLY_CURRENT_LIMIT = 40;
        public static final double EXTENDER_STATOR_CURRENT_LIMIT = 60;
        public static final double EXTENDER_SUPLLY_CURRENT_LIMIT = 50;

        //public static double CheckPoint = 2.646;
        public static double CheckPoint = 1.5;
    }

    public static class AimAprilTagCmdConstants{
        public static double LimeLightHeight = 0.22;
        public static double HeadingTorlerance = Math.toRadians(20.0);
        public static int missingThreshold = 5; 
    }

    // 转盘电机
    public static final class PathMotor {

        public static final int motorID = 7;
        public static final String canBusName = "rio";
        public static final double KP = 1;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        //TODOS CHECK
        public static final double speed = 70;
        public static final double voltage = 10;
    }
    public static final class FeedMotor {
        public static final int motorID = 6;
        public static final String canBusName = "rio";
        public static final double KP = 0.6;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        //public static final double speed = 2340/60*1.5;
        //TODOS CHECK
        public static final double speed = 40 *0.3;
                public static final double voltage = 5;

    }

    public static final class TurrentMotor {
        public static final int motorID = 0;
        public static final String canBusName = "rio";
        public static final double KP = 10;
        public static final double KI = 0.5;
        public static final double KD = 0.0;
        public static final double KS = 0.;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double speed = 1;
        public static final double minPos = -1;
        public static final double maxPos = 1;
    }
}
