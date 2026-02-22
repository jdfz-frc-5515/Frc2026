// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterEx;

/** An example command that uses an example subsystem. */
public class ShooterCmd extends Command {
    private final ShooterEx m_subsystem;

    // direction: 0 up 1 left 2 down 3 right
    public ShooterCmd(ShooterEx subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.startShooting();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopShooting();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}