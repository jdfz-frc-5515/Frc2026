package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Library.team19725.Point3D;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurrentSystem;
import frc.robot.utils.FindAprilTag;

/**
 * Command that keeps operator X/Y translation inputs but uses vision to compute
 * a desired robot heading (rotation) while active. When no vision target is
 * available, the driver's rotation stick is used.
 */
public class AimAprilTagCmd extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final TurrentSystem m_turret;
    private double m_desiredTurretAngle = 0.0;
    private boolean turnOnly = true;
    private boolean isFinishedFlag = false;
    // 连续未检测到目标的计数器；超过阈值则自动结束命令
    private int consecutiveMissing = 0;
    public AimAprilTagCmd(CommandSwerveDrivetrain drivetrain,
                        TurrentSystem turret,
                        boolean turnOnly) {
        m_drivetrain = drivetrain;
        m_turret = turret;
        this.turnOnly = turnOnly;
        isFinishedFlag = false;
        //不控制硬件，只读取&设置状态，故不addRequirements
        //addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        isFinishedFlag = false;
        m_drivetrain.resetRotationController();
        m_drivetrain.setUsingAutoAim(true);
        consecutiveMissing = 0;
        Pose2d pose = m_drivetrain.getPose();
        double robotHeading = pose.getRotation().getRadians();
        Point3D robotPos = new Point3D(pose.getX(), pose.getY(),Constants.AimAprilTagCmdConstants.LimeLightHeight);

        // 计算炮台在场地坐标系下的目标位姿
        Pose2d TurrentPose = m_turret.getTurretWorldPose(pose);
        // 提取炮台朝向角度（弧度）
        m_desiredTurretAngle = TurrentPose.getRotation().getRadians();

        double visionDesired = FindAprilTag.getTargetHeading(robotPos, robotHeading, m_desiredTurretAngle, turnOnly);
        visionDesired = 0;
        if (!Double.isNaN(visionDesired)) {
            // enable auto-aim only when we have a valid target
            m_drivetrain.resetRotationController();
            m_drivetrain.setUsingAutoAim(true);
            m_drivetrain.setDesiredAutoAimHeading(visionDesired);
            SmartDashboard.putNumber("TargetAprilTagHeading(Initing)", visionDesired);
        } else {
            consecutiveMissing = 1;
            SmartDashboard.putNumber("NoTargetAprilTagHeading", Double.NaN);
        }
        
    }

    @Override
    public void execute() {
        Pose2d pose = m_drivetrain.getPose();
        double robotHeading = pose.getRotation().getRadians();

        // Build 3D position for FindAprilTag (uses X,Y,Z)
        Point3D robotPos = new Point3D(pose.getX(), pose.getY(),Constants.AimAprilTagCmdConstants.LimeLightHeight);

        // Ask vision for desired heading (radians). May return NaN when no target.
        double visionDesired = FindAprilTag.getTargetHeading(robotPos, robotHeading, m_desiredTurretAngle, turnOnly);
        visionDesired = 0;
        if (!Double.isNaN(visionDesired)) {
            // enable auto-aim (in case it was off) and update target
            m_drivetrain.setUsingAutoAim(true);
            m_drivetrain.setDesiredAutoAimHeading(visionDesired);
            // reset missing counter when we have a good detection
            consecutiveMissing = 0;
            SmartDashboard.putNumber("TargetAprilTagHeading(executing)", visionDesired);
            SmartDashboard.putNumber("disAngle", Math.abs(FindAprilTag.normalizeAngle(robotHeading - visionDesired)));
            SmartDashboard.putNumber("tolerance", Constants.AimAprilTagCmdConstants.HeadingTorlerance);
            if (Math.abs(FindAprilTag.normalizeAngle(robotHeading - visionDesired)) < Constants.AimAprilTagCmdConstants.HeadingTorlerance) {
                isFinishedFlag = true;
            }
        }else {
            consecutiveMissing++;
            SmartDashboard.putNumber("NoTargetAprilTagHeading", Double.NaN);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.resetRotationController();
        m_drivetrain.setUsingAutoAim(false);
    }

    @Override
    public boolean isFinished() {
        // 结束条件：连续多帧未检测到 AprilTag 或 已经对准
        return (consecutiveMissing >= Constants.AimAprilTagCmdConstants.missingThreshold || 
                isFinishedFlag
        );
    }
}
