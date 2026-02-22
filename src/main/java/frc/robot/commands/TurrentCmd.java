// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeedingSubsystem;
import frc.robot.subsystems.TurrentSystem;

/** An example command that uses an example subsystem. */
public class TurrentCmd extends Command {
    private final TurrentSystem m_subsystem;
    private boolean m_isTurnRight = true;

    // direction: 0 up 1 left 2 down 3 right
    public TurrentCmd(TurrentSystem subsystem, boolean isTurnRight) {
        m_subsystem = subsystem;
        addRequirements(subsystem);

        m_isTurnRight = isTurnRight;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_isTurnRight) {
            m_subsystem.turnRight();
        }
        else {
            m_subsystem.turnLeft();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // System.out.println("end");
        m_subsystem.stopTurn();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
