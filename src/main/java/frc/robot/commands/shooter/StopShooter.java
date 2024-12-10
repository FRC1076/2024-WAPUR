package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StopShooter extends Command {
    private final ShooterSubsystem m_subsystem;
    
    public StopShooter(ShooterSubsystem subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize(){
        m_subsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}