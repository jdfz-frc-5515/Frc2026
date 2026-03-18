package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightModule;

public class WaitLLCmd extends Command {
    private boolean m_isOk = false;
    private int tick = -1;

    public WaitLLCmd() {

    }
    @Override
    public void initialize(){
    }
    @Override
    public void execute(){
        boolean isSeen = LimelightModule.getIsSeen();
        if (isSeen == false) {
            tick = -1;
            m_isOk = false;
        }
        else {
            if (tick < 0) {
                tick = 20;
            }
            tick--;
            if (tick == 0) {
                m_isOk = true;
            }
        }
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isOk;
    }
	
}
