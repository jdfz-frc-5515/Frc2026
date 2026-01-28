package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roller extends SubsystemBase {
    private TalonFX rollerMotor = new TalonFX(Constants.RollerConstants.MotorID);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    public double targetRpm = 60;
    public double Volt = 10;

    public Roller() {
        // 配置 PID + SVA 到 Slot0 
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = Constants.RollerConstants.kP;
        cfg.Slot0.kI = Constants.RollerConstants.kI;
        cfg.Slot0.kD = Constants.RollerConstants.kD;
        cfg.Slot0.kS = Constants.RollerConstants.kS;
        cfg.Slot0.kV = Constants.RollerConstants.kV;
        cfg.Slot0.kA = Constants.RollerConstants.kA;
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
        double rpm = rawVelocity * 60 / Constants.RollerConstants.GEAR_RATIO;
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
        double targetRPS = rpm * Constants.RollerConstants.GEAR_RATIO / 60;
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
