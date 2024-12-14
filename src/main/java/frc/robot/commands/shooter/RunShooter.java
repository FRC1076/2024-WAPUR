package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooter extends Command {
    private final ShooterSubsystem m_subsystem;
    
    public RunShooter(ShooterSubsystem subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize(){
        m_subsystem.runVolts(-8);
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
