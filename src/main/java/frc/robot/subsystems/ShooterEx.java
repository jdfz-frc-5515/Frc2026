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
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = isPrimary ? InvertedValue.Clockwise_Positive: InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = lockMotor ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        config.Slot0.kP = Constants.ShooterConstants.KP;
        config.Slot0.kI = Constants.ShooterConstants.KI;
        config.Slot0.kD = Constants.ShooterConstants.KD;
        config.Slot0.kS = Constants.ShooterConstants.KS;
        config.Slot0.kV = Constants.ShooterConstants.KV;
        config.Slot0.kA = Constants.ShooterConstants.KA;
        
        // config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // config.CurrentLimits.SupplyCurrentLimit = 1;
        // config.CurrentLimits.SupplyCurrentLowerLimit
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 100;
        return config;
    }
    double speed_dt = 0;
    double DT = 3;

    public void init() {
        m_primary.getConfigurator().apply(getMotorConfiguration(true, false));
        m_follower.getConfigurator().apply(getMotorConfiguration(false, false));
        m_follower.setControl(new Follower(m_primary.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void incSpeed() {
        speed_dt += DT;
    }
    public void decSpeed() {
        speed_dt -= DT;
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

    double startTimeCount = 0;
    boolean isAtMaxSpeed = false;
    public boolean getIsAtMaxSpeed() {
        return isAtMaxSpeed;
    }
    
    public void update(){
        double spd = targetSpeed + speed_dt;
        if (isShooting) {
            SmartDashboard.putNumber("shooting speed", spd);
            velocityRequest.Velocity = spd;
            m_primary.setControl(velocityRequest);
        }
        else {
            velocityRequest.Velocity = Constants.ShooterConstants.idleSpeed;
            m_primary.stopMotor();
            isAtMaxSpeed = false;
            startTimeCount = 0;
        }

        if (m_primary.getMotorVoltage().getValueAsDouble() >= (spd / 10)) {
            startTimeCount++;
            if (startTimeCount >= 20) {
                isAtMaxSpeed = true;
            }
        }
        else {
            isAtMaxSpeed = false;
            startTimeCount = 0;
        }

        SmartDashboard.putNumber("ShootingSpeed", m_primary.getMotorVoltage() .getValueAsDouble());
        SmartDashboard.putBoolean("isShooting", isShooting);
    }
}
