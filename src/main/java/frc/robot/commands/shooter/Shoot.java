package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class Shoot extends Command {
    private final ShooterSubsystem m_subsystem;

    public Shoot(ShooterSubsystem subsystem) {
        m_subsystem = subsystem;
    }

    public void initialize() {
        m_subsystem.runVolts(ShooterConstants.shooterVoltage);
    }

    public boolean isFinished() {
        return true;
    }
}
