package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.MessageSender;

public class ManualExtenderCmd extends Command{
    private IntakeSubsystem intakeSubsystem;
    private boolean ifIn = false;
    public ManualExtenderCmd(IntakeSubsystem m_IntakeSubsystem, boolean ifIn){
        intakeSubsystem = m_IntakeSubsystem;
        this.ifIn = ifIn;
    }
    @Override
    public void initialize(){
        if(ifIn){
            intakeSubsystem.setExtenderVoltage(-2);
        }
       else{
            intakeSubsystem.setExtenderVoltage(1);
        }
    }
    @Override
    public void execute(){
        if(ifIn){
            intakeSubsystem.setExtenderVoltage(-2);
        }
       else{
            intakeSubsystem.setExtenderVoltage(1);
        }
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setExtenderVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
