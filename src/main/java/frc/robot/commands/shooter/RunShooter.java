package frc.robot.commands.shooter;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooter extends Command {

    private final ShooterSubsystem m_shooter;

    public RunShooter(ShooterSubsystem shooter){
        m_shooter = shooter;
    }

    @Override
    public void initialize(){
        m_shooter.runVolts(10);
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
