package frc.robot.commands.turret;

import frc.robot.subsystems.turret.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

class TurnTurretOpenLoop extends Command {

    private final TurretSubsystem m_subsystem;
    private final DoubleSupplier voltSupplier;

    public TurnTurretOpenLoop(DoubleSupplier voltSupplier, TurretSubsystem subsystem){
        m_subsystem = subsystem;
        this.voltSupplier = voltSupplier;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_subsystem.turnVolts(voltSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished(){
        return false;
    }
}