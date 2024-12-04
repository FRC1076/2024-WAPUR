package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class SetElevatorVelocity extends Command {
    private final ElevatorSubsystem m_subsystem;
    private final DoubleSupplier speedSupplier; // All speeds are in meters per second

    public SetElevatorVelocity(ElevatorSubsystem subsystem, DoubleSupplier speedSupplier){
        m_subsystem = subsystem;
        this.speedSupplier = speedSupplier;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        m_subsystem.setVelocity(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.setVelocity(0);
    }


}
