package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurrentSystem;
import frc.robot.subsystems.ShooterEx;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Library.team1706.FieldRelativeSpeed;
import frc.robot.Library.team1706.FieldRelativeAccel;
import frc.robot.Library.team1706.LinearInterpolationTable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartShootCmd extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final TurrentSystem m_turret;
    private final ShooterEx m_shooter;
    private final LinearInterpolationTable distRpsTable = ShooterConstants.kRPMTable;
    private final LinearInterpolationTable distShotTimeTable = ShooterConstants.kShotTimeTable;
    private final Translation2d hubLocation = ShooterConstants.targetHub;
    private int max_iteration = 1;
    private double accComp = 0.100;
    public SmartShootCmd(CommandSwerveDrivetrain drivetrain, TurrentSystem turret, ShooterEx shooter) {
        this.m_drivetrain = drivetrain;
        this.m_turret = turret;
        this.m_shooter = shooter;
    }
    @Override
    public void initialize() {
        m_turret.startAiming(true);
    }
    
    @Override
    public void execute() {
        m_turret.startAiming(true);
        Translation2d virtualTarget = hubLocation;
        for (int i=0; i < max_iteration; i++) {
            double shotDistance = virtualTarget.getDistance(m_turret.getTurretWorldPose(m_drivetrain.getPose()).getTranslation());
            double shotTime = distShotTimeTable.getOutput(shotDistance);
            FieldRelativeSpeed driveFieldSpeed = m_drivetrain.getFieldRelativeSpeed();
            FieldRelativeAccel driveFieldAccel = m_drivetrain.getFieldRelativeAccel();
            Translation2d targetShiftVector = new Translation2d(-shotTime*(driveFieldSpeed.vx+driveFieldAccel.ax*accComp), -shotTime*((driveFieldSpeed.vy+driveFieldAccel.ay*accComp)));
            virtualTarget = virtualTarget.plus(targetShiftVector);
            // Translation2d toVirtualTargetVector = virtualTarget.minus(m_drivetrain.getPose().getTranslation());
            // double newShotTime = distShotTimeTable.getOutput(toVirtualTargetVector.getNorm());
            // if (Math.abs(newShotTime - shotTime) < 0.010) {
            //     i = 4;
            // }
            // shotTime = newShotTime;
        }
        m_turret.setTarget(virtualTarget);
        double calc_deviation = virtualTarget.getDistance(m_turret.getTurretWorldPose(m_drivetrain.getPose()).getTranslation());
        double virtual_rps = distRpsTable.getOutput(calc_deviation);
        m_shooter.setTargetSpeed(distRpsTable.getOutput(virtual_rps));
    }
    
    @Override
    public void end(boolean interrupted) {
        m_turret.stopAiming();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
