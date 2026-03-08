package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.SmartDashboardEx;

public class IntakeCmd extends Command{
	private boolean hasIntakeReachTarget = false;
	private boolean hasIntakeStop = false;
	private boolean hasSupplierPressed = false;
	private final boolean ifIn;
	private final IntakeSubsystem intakeSubsystem;
	private final double targetPosition;
	private static final double TOLERANCE = Constants.IntakeConstants.Tolerance;
	private final BooleanSupplier supplier;
	public static double InPosition = Constants.IntakeConstants.IN_POS;
	public static double OutPosition = Constants.IntakeConstants.OUT_POS;
	public static double CheckPoint = 2.646;
	public IntakeCmd(IntakeSubsystem m_IntakeSubsystem, boolean ifIn, BooleanSupplier m_Supplier) {
		this.ifIn = ifIn;
		this.intakeSubsystem = m_IntakeSubsystem;
		this.supplier = m_Supplier;
		if(ifIn){
			this.targetPosition = InPosition;
		}
		else{
			this.targetPosition = OutPosition;
		}
		addRequirements(m_IntakeSubsystem);
	}

	@Override
	public void initialize() {
		hasIntakeReachTarget = false;
		if(hasReachedTarget()){
			hasIntakeReachTarget = true;
		}
		if(ifIn){
			hasIntakeStop = true;
		}
		else{
			hasIntakeStop = false;
		}
		
	}

	@Override
	public void execute() {
		if(supplier.getAsBoolean() == true && hasSupplierPressed == false){
			hasIntakeStop = !hasIntakeStop;
			hasSupplierPressed = true;
		}
		if(supplier.getAsBoolean() == false){
			hasSupplierPressed = false;
		}
		if(!hasIntakeStop){
			intakeSubsystem.setInatkeVoltage(Constants.IntakeConstants.Intake_Voltage);
		}
		else{
			intakeSubsystem.setInatkeVoltage(0);
		}
		double now = intakeSubsystem.getCurrentExtenderPosition();
		SmartDashboardEx.putBoolean("hasIntakeOut", hasIntakeReachTarget);
		SmartDashboard.putNumber("MotorPos", now);
		if(!hasIntakeReachTarget){
			if (now < targetPosition) {
				intakeSubsystem.setExtenderVoltage(Constants.IntakeConstants.Extender_Voltage);
			} else if (now > targetPosition) {
				if(now > CheckPoint){
					intakeSubsystem.setExtenderVoltage(-Constants.IntakeConstants.Extender_Push_Voltage);
				}
				else{
					intakeSubsystem.setExtenderVoltage(-Constants.IntakeConstants.Extender_Voltage);
				}
			}
		}
		if(hasReachedTarget()){
			hasIntakeReachTarget = true;
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.setExtenderVoltage(0);
		intakeSubsystem.setInatkeVoltage(0);
	}
	private boolean hasReachedTarget(){
		return (Math.abs(targetPosition - intakeSubsystem.getCurrentExtenderPosition()) <= TOLERANCE); 
	}
}
