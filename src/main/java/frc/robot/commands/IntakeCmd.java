package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.MessageSender;

public class IntakeCmd extends Command{
	private IntakeSubsystem intakeSubsystem;
    public IntakeCmd(IntakeSubsystem m_IntakeSubsystem){
        intakeSubsystem = m_IntakeSubsystem;
    }
    @Override
    public void initialize(){
        intakeSubsystem.toggleIntakeMode();
    }
    @Override
    public void execute(){
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
	
}
