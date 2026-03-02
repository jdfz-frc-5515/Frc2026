package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterEx {
    public ShooterEx() {
        init();
    }
    private final CANBus shooterCanBus = new CANBus(Constants.ShooterConstants.canBusName);
    private final TalonFX m_primary = new TalonFX(Constants.ShooterConstants.PRIMARY_CAN_ID, shooterCanBus);
    private final TalonFX m_follower = new TalonFX(Constants.ShooterConstants.FOLLOWER_CAN_ID, shooterCanBus);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private boolean isShooting = false;
    private double targetSpeed = Constants.ShooterConstants.shootingSpeed;

    private TalonFXConfiguration getMotorConfiguration(boolean isPrimary, boolean lockMotor) {
        TalonFXConfiguration shooterConfiguration = new TalonFXConfiguration();

        shooterConfiguration.MotorOutput.Inverted = isPrimary ? InvertedValue.Clockwise_Positive: InvertedValue.CounterClockwise_Positive;
        shooterConfiguration.MotorOutput.NeutralMode = lockMotor ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        shooterConfiguration.Slot0.kP = Constants.ShooterConstants.KP;
        shooterConfiguration.Slot0.kI = Constants.ShooterConstants.KI;
        shooterConfiguration.Slot0.kD = Constants.ShooterConstants.KD;
        shooterConfiguration.Slot0.kS = Constants.ShooterConstants.KS;
        shooterConfiguration.Slot0.kV = Constants.ShooterConstants.KV;
        shooterConfiguration.Slot0.kA = Constants.ShooterConstants.KA;
        return shooterConfiguration;
    }
    public void init() {
        m_primary.getConfigurator().apply(getMotorConfiguration(true, false));
        m_follower.getConfigurator().apply(getMotorConfiguration(false, false));
        m_follower.setControl(new Follower(m_primary.getDeviceID(), MotorAlignmentValue.Opposed));
    }
    public void setTargetSpeed(double speed) {
        // targetSpeed = Math.min(speed, Constants.ShooterConstants.shootingSpeed);
        targetSpeed=speed;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void startShooting() {
        isShooting = true;
    }
    public void stopShooting() {
        isShooting = false;
    }

    public boolean getIsShooting() {
        return isShooting;
    }
    public void toggleShooting() {
        isShooting = !isShooting;
    }
    // @Override
    // public void periodic() {
    //     update();
    // }

    public void update(){
        if (isShooting) {
            SmartDashboard.putNumber("shooting speed", targetSpeed);
            velocityRequest.Velocity = targetSpeed;
            m_primary.setControl(velocityRequest);
        }
        else {
            velocityRequest.Velocity = Constants.ShooterConstants.idleSpeed;
            m_primary.stopMotor();
        }
        
        SmartDashboard.putBoolean("isShooting", isShooting);
    }
}
