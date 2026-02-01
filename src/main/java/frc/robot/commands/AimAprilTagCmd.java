package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Library.team19725.Point3D;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.utils.FindAprilTag;

/**
 * Command that keeps operator X/Y translation inputs but uses vision to compute
 * a desired robot heading (rotation) while active. When no vision target is
 * available, the driver's rotation stick is used.
 */
public class AimAprilTagCmd extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final ImprovedCommandXboxController m_controller;
    private final PIDController m_headingPid;
    
    private final double m_maxAngular = Constants.AimAprilTagConstants.m_maxAngular;
    private final double LimeLightHeight = Constants.AimAprilTagConstants.LimeLightHeight;
    
    private static final double Kp = Constants.AimAprilTagConstants.Kp;
    private static final double Ki = Constants.AimAprilTagConstants.Ki;
    private static final double Kd = Constants.AimAprilTagConstants.Kd;
    
    private double m_desiredTurretAngle = 0.0;
    private boolean turnOnly = true;

    public AimAprilTagCmd(CommandSwerveDrivetrain drivetrain,
                        ImprovedCommandXboxController controller,
                        double desiredTurretAngle,
                        boolean turnOnly) {
        m_drivetrain = drivetrain;
        m_controller = controller;
        m_desiredTurretAngle = desiredTurretAngle;
        this.turnOnly = turnOnly;

        m_headingPid = new PIDController(Kp, Ki, Kd); //TODO tune PID values
        m_headingPid.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_headingPid.reset();
        Pose2d pose = m_drivetrain.getPose();
        double robotHeading = pose.getRotation().getRadians();
        Point3D robotPos = new Point3D(pose.getX(), pose.getY(),LimeLightHeight);
        double visionDesired = FindAprilTag.getTargetHeading(robotPos, robotHeading, m_desiredTurretAngle, turnOnly);
        SmartDashboard.putNumber("TargetAprilTagHeading", visionDesired);
    }

    @Override
    public void execute() {
        Pose2d pose = m_drivetrain.getPose();
        double robotHeading = pose.getRotation().getRadians();

        // Build 3D position for FindAprilTag (uses X,Y,Z)
        Point3D robotPos = new Point3D(pose.getX(), pose.getY(),LimeLightHeight);

        // Ask vision for desired heading (radians). May return NaN when no target.
        double visionDesired = FindAprilTag.getTargetHeading(robotPos, robotHeading, m_desiredTurretAngle, turnOnly);

        double omegaCmd;
        if (!Double.isNaN(visionDesired)) {
            omegaCmd = m_headingPid.calculate(robotHeading, visionDesired);
            omegaCmd = MathUtil.clamp(omegaCmd, -m_maxAngular, m_maxAngular);
            m_drivetrain.driveFieldCentric(m_controller, omegaCmd);
        } else {
            m_drivetrain.driveFieldCentric(m_controller);
        }
        SmartDashboard.putNumber("TargetAprilTagHeading", visionDesired);
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing special; default command will resume
    }

    @Override
    public boolean isFinished() {
        return false; // runs while trigger is held (bound via whileTrue)
    }
}
