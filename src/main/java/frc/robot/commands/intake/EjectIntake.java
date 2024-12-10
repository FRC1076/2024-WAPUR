package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class EjectIntake extends Command {
    private final IntakeSubsystem m_subsystem;
    private Timer timer = new Timer();

    public EjectIntake(IntakeSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.runVolts(4.5);
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