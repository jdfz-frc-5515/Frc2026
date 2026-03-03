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
    private int max_iteration = 5;
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
        // Get drive speed, acc, and translation
        FieldRelativeSpeed driveFieldSpeed = m_drivetrain.getFieldRelativeSpeed();
        FieldRelativeAccel driveFieldAccel = m_drivetrain.getFieldRelativeAccel();
        Translation2d drivetrainTranslation = m_drivetrain.getPose().getTranslation();
        Translation2d turretWorldTranslation = m_turret.getTurretWorldPose(m_drivetrain.getPose()).getTranslation();
        FieldRelativeSpeed turretSpeed = m_turret.getTurretSpeed(driveFieldSpeed);
        // Get initial virtual shot distance and time
        double shotDistance = virtualTarget.getDistance(turretWorldTranslation);
        double shotTime = distShotTimeTable.getOutput(shotDistance);
        // Iterate to get better shot time and target
        for (int i=0; i < max_iteration; i++) {
            // Calculate new virtual shot target
            Translation2d targetShiftVector = new Translation2d(
                -shotTime * (turretSpeed.getX() + driveFieldAccel.ax * accComp), 
                -shotTime * (turretSpeed.getY() + driveFieldAccel.ay * accComp));
            virtualTarget = hubLocation.plus(targetShiftVector);
            // Calculate new virtual shot time
            Translation2d toVirtualTargetVector = virtualTarget.minus(drivetrainTranslation);
            double newShotTime = distShotTimeTable.getOutput(toVirtualTargetVector.getNorm());
            // If time converge, break the loop
            if (Math.abs(newShotTime - shotTime) < 0.010) {
                shotTime = newShotTime;
                break;
            }
            // Update shot time
            shotTime = newShotTime;
        }
        double calc_deviation = virtualTarget.getDistance(drivetrainTranslation);
        SmartDashboard.putNumber("deviation", calc_deviation);
        m_turret.setTarget(virtualTarget);
        m_shooter.setTargetSpeed(distRpsTable.getOutput(calc_deviation));
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
