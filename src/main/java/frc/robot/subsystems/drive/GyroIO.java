package frc.robot.subsystems.drive;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs{
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public Measure<Velocity<Angle>> yawVelocity = RadiansPerSecond.of(0);
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void resetHeading() {}
}