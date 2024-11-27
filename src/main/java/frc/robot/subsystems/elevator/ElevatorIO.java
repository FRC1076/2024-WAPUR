package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Distance;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {

        public double appliedVolts = 0;
        public double leadCurrentAmps = 0;
        public double followCurrentAmps = 0;

        public double elevatorHeightMeters = 0;
        public double velocityMetersPerSecond = 0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setPosition(double positionMeters) {}

    public default void setPosition(Measure<Distance> positionMeters) {}
    
    public default void setVelocity(double velocityMetersPerSecond) {}

    public default void setVelocity(Measure<Velocity<Distance>> velocity) {
        setVelocity(velocity.in(MetersPerSecond));
    }

    public default void setVoltage(double volts) {}

    public default void setVoltage(Measure<Voltage> voltage) {
        setVoltage(voltage.in(Volts));
    }
}
