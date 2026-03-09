package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import java.security.MessageDigest;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.MessageSender;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.SmartDashboardEx;

public class IntakeSubsystem extends SubsystemBase {
    private final int extenderMotorCANID = IntakeConstants.EXTENDER_CAN_ID; 
    private final int intakeMotorCANID = IntakeConstants.INTAKE_CAN_ID;
    private final CANBus MotorCANBus= new CANBus("rio");
    private final TalonFX ExtenderMotor;
    private final TalonFX IntakeMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    // PID S V A constants for extender slot0 are defined in Constants.IntakeConstants
    private static final double EXT_KP = IntakeConstants.EXTENDER_KP;
    private static final double EXT_KI = IntakeConstants.EXTENDER_KI;
    private static final double EXT_KD = IntakeConstants.EXTENDER_KD;
    private static final double EXT_KS = IntakeConstants.EXTENDER_KS;
    private static final double EXT_KV = IntakeConstants.EXTENDER_KV;
    private static final double EXT_KA = IntakeConstants.EXTENDER_KA;
    private static final double INTAKE_KP = IntakeConstants.INTAKE_KP;
    private static final double INTAKE_KI = IntakeConstants.INTAKE_KI;
    private static final double INTAKE_KD = IntakeConstants.INTAKE_KD;
    private static final double INTAKE_KS = IntakeConstants.INTAKE_KS;
    private static final double INTAKE_KV = IntakeConstants.INTAKE_KV;
    private static final double INTAKE_KA = IntakeConstants.INTAKE_KA;
    // Jam Detector
    private final Debouncer jamDebouncer = new Debouncer(0.2, DebounceType.kRising);
    private final double jamCurrentThreshold = 100.0;
    //state machine
    private boolean runIntakeMode = false;
    //true: in false: out
    private boolean runExtenderMode = true;
    public void toggleIntakeMode(){
        MessageSender.log("222222222222222222222222222222222");
        runIntakeMode = !runIntakeMode;
    }
    public void toggleExtenderMode(){
        runExtenderMode = !runExtenderMode;
    }
    private double targetPosition = 0;
    private boolean hasIntakeReachTarget = false;

    private static final double TOLERANCE = Constants.IntakeConstants.Tolerance;
	public static double CheckPoint = 2.646;

    private final TalonFXConfiguration ExtenderConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(100))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(100))
                .withSupplyCurrentLimitEnable(true)
        )
        .withSlot0(
            new Slot0Configs()
                .withKP(EXT_KP)
                .withKI(EXT_KI)
                .withKD(EXT_KD)
                .withKS(EXT_KS)
                .withKV(EXT_KV)
                .withKA(EXT_KA)
        );
    private final TalonFXConfiguration IntakeConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(30))
                .withSupplyCurrentLimitEnable(true)
        )
        .withSlot0(
            new Slot0Configs()
                .withKP(INTAKE_KP)
                .withKI(INTAKE_KI)
                .withKD(INTAKE_KD)
                .withKS(INTAKE_KS)
                .withKV(INTAKE_KV)
                .withKA(INTAKE_KA)
        );
    public IntakeSubsystem() {
        ExtenderMotor = new TalonFX(extenderMotorCANID, MotorCANBus);
        IntakeMotor = new TalonFX(intakeMotorCANID, MotorCANBus);
        ExtenderMotor.getConfigurator().apply(ExtenderConfig);
        IntakeMotor.getConfigurator().apply(IntakeConfig);
        ExtenderMotor.setPosition(0);
    }

    @Override
    public void periodic(){
        update();
        if (haveObstacle()
            || ExtenderMotor.getPosition().getValueAsDouble() < -1 
            || ExtenderMotor.getPosition().getValueAsDouble() > 6) {
            ExtenderMotor.stopMotor();
        }
    }

    public void setExtenderVoltage(double voltage) {
        if(MiscUtils.compareDouble(voltage, 0)){
            MessageSender.log("****************************");
            ExtenderMotor.stopMotor();
        }
        else{
            MessageSender.log("888888888888888888888888888888888");
            ExtenderMotor.setControl(
            voltageRequest
                .withOutput(Volts.of(voltage))
            );
        }
        
    }

    public boolean haveObstacle() {
        double currentAmps = ExtenderMotor.getStatorCurrent().getValueAsDouble();
        boolean overThreshold = currentAmps > jamCurrentThreshold;
        return jamDebouncer.calculate(overThreshold);
    }

    public double getCurrentExtenderPosition(){
        return ExtenderMotor.getPosition().getValueAsDouble();
    }
    public void resetExtenderPosition(double position){
        ExtenderMotor.setPosition(position);
    }

    
    public void setInatkeVoltage(double Voltage){
        if(MiscUtils.compareDouble(Voltage, 0)){
            IntakeMotor.stopMotor();
        }
        else{
            IntakeMotor.setControl(
            voltageRequest
                .withOutput(Volts.of(Voltage))
            );
        }
        
    }
    
    public void update(){
        if(hasReachedTarget()){
			hasIntakeReachTarget = true;
		}
        setTargetPoint();

        if(runIntakeMode){
            setInatkeVoltage(Constants.IntakeConstants.Intake_Voltage);
        }
        else{
            setInatkeVoltage(0);
        }

            double now = getCurrentExtenderPosition();

		    SmartDashboardEx.putBoolean("hasIntakeReachTarget", hasIntakeReachTarget);
		    SmartDashboard.putBoolean("runIntakeMode", runIntakeMode);
		    SmartDashboard.putNumber("MotorPos", now);
            SmartDashboard.putNumber("targetPOS", targetPosition);
		    if(!hasReachedTarget()){
                MessageSender.log("29292929292929929292");
			    if (now < targetPosition) {
				    setExtenderVoltage(Constants.IntakeConstants.Extender_Voltage);
			    } else if (now > targetPosition) {
				    if(now > CheckPoint){
					    setExtenderVoltage(-Constants.IntakeConstants.Extender_Push_Voltage);
				    }
				    else{
					    setExtenderVoltage(-Constants.IntakeConstants.Extender_Voltage);
				    }
			    }
		    }
            else{
                MessageSender.log("3939393939939393939393");
                ExtenderMotor.stopMotor();
            }
    }
    private boolean hasReachedTarget(){
		return (Math.abs(targetPosition - getCurrentExtenderPosition()) <= TOLERANCE); 
	}
    private void setTargetPoint(){
		if(runExtenderMode){
			targetPosition = Constants.IntakeConstants.IN_POS;
		}
		else{
			targetPosition = Constants.IntakeConstants.OUT_POS;
		}
	}
}
