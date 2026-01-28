package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
    public static class RollerConstants {
        public static final int MotorID = 100;
        public static final double kOpenPower = 0.8;

        // 编码器与换算常量（根据实际电机/齿比调整）
        public static final double GEAR_RATIO = 27.0; // 机械减速比：输出轴转一圈编码器转几圈（若为 10:1，填 10）


        //6  51.5
        //8  69
        //10  86
        //12  103.5
        // PID + SVA 常量
        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.04;   
        public static final double kV = 0.115; 
        public static final double kA = 0.00000; 
    }


    private TalonFX rollerMotor = new TalonFX(RollerConstants.MotorID);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    public double targetRpm = 60;
    public double Volt = 10;

    public Roller() {
        // 配置 PID + SVA 到 Slot0 
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = RollerConstants.kP;
        cfg.Slot0.kI = RollerConstants.kI;
        cfg.Slot0.kD = RollerConstants.kD;
        cfg.Slot0.kS = RollerConstants.kS;
        cfg.Slot0.kV = RollerConstants.kV;
        cfg.Slot0.kA = RollerConstants.kA;
        // 若使用远端/其它传感器，或需要调整 SensorToMechanismRatio/ RotorToSensorRatio，
        // 在这里设置：cfg.Feedback.SensorToMechanismRatio = ...;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotor.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Volt", Volt);
        SmartDashboard.putNumber("TargetRPM", targetRpm);
        // SmartDashboard.putNumber("StatorCurrent", rollerMotor.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("SupplyCurrent", rollerMotor.getSupplyCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("TorqueCurrent", rollerMotor.getTorqueCurrent().getValueAsDouble());
        double rawVelocity = rollerMotor.getVelocity().getValueAsDouble();
        // SmartDashboard.putNumber("Roller Raw Velocity (rps)", rawVelocity);
        double rpm = rawVelocity * 60 / RollerConstants.GEAR_RATIO;
        SmartDashboard.putNumber("Real Roller RPM", rpm);
    }

    public void setOpen(boolean open) {
        if (open) {
            rollerMotor.setVoltage(Volt);
        } else {
            rollerMotor.setVoltage(0);
        }
    }
    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        double targetRPS = rpm * RollerConstants.GEAR_RATIO / 60;
        rollerMotor.setControl(velocityControl.withVelocity(targetRPS).withSlot(0));
    }
    public void setTargetRpm(){
        setTargetRpm(targetRpm);
    }
    public void powerplus(){
        Volt += 1;
    }
    public void powerminus(){
        Volt -= 1;
    }
    public void speedplus(){
        targetRpm += 10;
        setTargetRpm();
    }
    public void speedminus(){
        targetRpm -= 10;
        setTargetRpm();
    }
}
