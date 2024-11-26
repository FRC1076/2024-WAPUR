package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootStop extends Command{
    private final ShooterSubsystem m_subsystem;

    public ShootStop(ShooterSubsystem subsystem) {
        m_subsystem = subsystem;
    }

    public void initialize() {
        m_subsystem.runVolts(0);
    }

    public boolean isFinished() {
        return true;
    }
}
