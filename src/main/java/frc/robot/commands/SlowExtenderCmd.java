package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Extender.ExtenderSpeed;

public class SlowExtenderCmd extends Command{
	private final Extender extender;
	private final double targetPosition;
	private static final double TOLERANCE = 1.0;

	public SlowExtenderCmd(Extender extender, double targetPosition) {
		this.extender = extender;
		this.targetPosition = targetPosition;
		addRequirements(extender);
	}

	@Override
	public void initialize() {
		// no-op: ensure motor is stopped until execute runs (use Extender methods if available)
	}

	@Override
	public void execute() {
		double now = extender.getCurrentMotorPosition();
		if (now < targetPosition) {
			extender.setVelocity(ExtenderSpeed.SLOWOUT);
		} else if (now > targetPosition) {
			extender.setVelocity(ExtenderSpeed.SLOWIN);
		}
	}

	@Override
	public boolean isFinished() {
		double now = extender.getCurrentMotorPosition();
		return Math.abs(targetPosition - now) <= TOLERANCE;
	}

	@Override
	public void end(boolean interrupted) {
		extender.setPercentOutput(0);
	}
}
