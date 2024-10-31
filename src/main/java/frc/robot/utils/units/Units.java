package frc.robot.utils.units;
import edu.wpi.first.units.Angle;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Velocity;

/** Defines units that are not defined in the default WPILib Units class. Is designed to be statically imported*/
public final class Units {
    public static final Velocity<Velocity<Angle>> RadiansPerSecondSquared = RadiansPerSecond.per(Second);
    public static final Velocity<Velocity<Angle>> RPMPerSecond = RPM.per(Second);
}
