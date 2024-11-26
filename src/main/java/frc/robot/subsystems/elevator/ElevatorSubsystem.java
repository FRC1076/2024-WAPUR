package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io){
        this.io = io;
    }

    public void setPosition(double positionMeters) {
        io.setPosition(positionMeters);
    }

    public void setPosition(Measure<Distance> positionMeters) {
        io.setPosition(positionMeters);
    }

    public void setVelocity(double velocityMetersPerSecond){
        io.setVelocity(velocityMetersPerSecond);
    }

    public void setVelocity(Measure<Velocity<Distance>> velocity){
        io.setVelocity(velocity);
    }

    /** Returns position of the elevator, as a Measure<Distance> */
    public Measure<Distance> getPosition(){
        return Meters.of(inputs.elevatorHeightMeters);
    }

    /** Returns position of the elevator, as a double */
    public double getPositionMeters(){
        return inputs.elevatorHeightMeters;
    }


    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator",inputs);
    }
}
