package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Library.team1706.LinearInterpolationTable;

public class ShootRPSManager extends SubsystemBase {
    private static ShootRPSManager instance = null;
    private ShooterEx shooter;
    private CommandSwerveDrivetrain m_drivetrain;
    private TurrentSystem m_turret;
    private final LinearInterpolationTable m_rpsTable = Constants.ShooterConstants.kRPMTable;
    
    private double currentDistance = 0.0;
    private double lastShooterSpeed = 0.0;
    
    private ShootRPSManager() {
        // Private constructor for singleton
    }

    public static ShootRPSManager getInstance() {
        if (instance == null) {
            instance = new ShootRPSManager();
        }
        return instance;
    }

    public void setShooter(ShooterEx shooter) {
        this.shooter = shooter;
    }
    public void setSwerve(CommandSwerveDrivetrain drivetrain) {
        this.m_drivetrain = drivetrain;
    }
    public void setTurret(TurrentSystem turret) {
        this.m_turret = turret;
    }

    public void setDistance(double distance) {
        if (distance > 0.1) {
        this.currentDistance = distance;
        }
    }

    public double getCalculatedSpeed() {
        return m_rpsTable.getOutput(currentDistance);
    }

    @Override
    public void periodic() {
        if (shooter == null) return;
        // setDistance(LimelightHelpers.getBotPose2d_wpiBlue("limelight-left").getTranslation().getDistance(Constants.ShooterConstants.targetHub));
        setDistance(m_turret.getTurretWorldPose(m_drivetrain.getPose()).getTranslation().getDistance(Constants.ShooterConstants.targetHub));
        // Calculate target speed based on distance
        double targetSpeed = getCalculatedSpeed();

        // Prevent shooter oscillation
        if( Math.abs(targetSpeed - lastShooterSpeed) < Constants.ShooterConstants.kPreventShooterOscilliationRPS ){
            targetSpeed = lastShooterSpeed;
        }else{
            lastShooterSpeed = targetSpeed;
        }

        // Push to SmartDashboard for debugging
        SmartDashboard.putNumber("ShootRPSManager/TargetSpeed", targetSpeed);
        SmartDashboard.putNumber("ShootRPSManager/CurrentDistance", currentDistance);
        
        // Update the shooter subsystem
        shooter.setTargetSpeed(targetSpeed);
    }
}
