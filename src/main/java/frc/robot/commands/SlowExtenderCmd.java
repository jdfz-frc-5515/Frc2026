package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class SlowExtenderCmd extends Command{
	private boolean ifIn;
	private final IntakeSubsystem intakeSubsystem;
	private final double targetPosition;
	private static final double TOLERANCE = Constants.IntakeConstants.Tolerance;
	public static double InPosition = Constants.IntakeConstants.IN_POS;
	public static double OutPosition = Constants.IntakeConstants.OUT_POS;
	public SlowExtenderCmd(IntakeSubsystem m_IntakeSubsystem, boolean ifIn) {
		this.ifIn = ifIn;
		this.intakeSubsystem = m_IntakeSubsystem;
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
	}

	@Override
	public void execute() {
		double now = intakeSubsystem.getCurrentExtenderPosition();
		SmartDashboard.putNumber("MotorPos", now);
		if(intakeSubsystem.haveObstacle()){
			intakeSubsystem.setExtenderVoltage(0);
			return; // Stop if an obstacle is detected
		}
		if (now < targetPosition) {
			intakeSubsystem.setExtenderVoltage(Constants.IntakeConstants.Extender_Voltage);
		} else if (now > targetPosition) {
			intakeSubsystem.setExtenderVoltage(-Constants.IntakeConstants.Extender_Voltage);
		}
	}

	@Override
	public boolean isFinished() {
		double now = intakeSubsystem.getCurrentExtenderPosition();
		return Math.abs(targetPosition - now) <= TOLERANCE;
	}

	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.setExtenderVoltage(0);
	}
}
