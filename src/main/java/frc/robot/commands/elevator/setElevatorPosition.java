package frc.robot.commands.elevator;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import edu.wpi.first.math.controller.PIDController;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import static frc.robot.Constants.Elevator.PositionControl.*;

public class setElevatorPosition extends PIDCommand {
    public setElevatorPosition(double setpointMeters, ElevatorSubsystem elevator){
        super(
            new PIDController(kP,kI,kD), 
            elevator::getPositionMeters, 
            () -> setpointMeters,
            output -> elevator.setVelocity(output), 
            elevator
        );
    }

    @Override
    public boolean isFinished(){
        return getController().atSetpoint();
    }
}
