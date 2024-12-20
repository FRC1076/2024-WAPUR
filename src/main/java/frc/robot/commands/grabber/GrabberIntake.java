package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class GrabberIntake extends Command {
    private final GrabberSubsystem m_subsystem;

    public GrabberIntake(GrabberSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }
    
    @Override
    public void initialize() {
        m_subsystem.runVolts(6);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        //return true;
        return false;
    }
}
