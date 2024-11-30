package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class GrabberEject extends Command {
    private final GrabberSubsystem m_subsystem;
    private Timer timer = new Timer();
    public GrabberEject(GrabberSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.runVolts(2);
        timer.restart();
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}