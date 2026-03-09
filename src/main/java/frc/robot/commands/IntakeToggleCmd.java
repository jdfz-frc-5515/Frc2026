package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem2;
import frc.robot.utils.MessageSender;
import frc.robot.utils.SmartDashboardEx;

public class IntakeToggleCmd extends Command{
	private IntakeSubsystem2 m_IntakeSubsystem;
	public IntakeToggleCmd(IntakeSubsystem2 intakeSubsystem) {
		m_IntakeSubsystem = intakeSubsystem;
		addRequirements(m_IntakeSubsystem);
	}

	@Override
	public void initialize() {
		m_IntakeSubsystem.toggleExtender();
	}

	@Override
	public void execute() {
		MessageSender.log("IntakeToggleCmd update");
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
	}
}
