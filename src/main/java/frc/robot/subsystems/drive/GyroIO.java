package frc.robot.subsystems.drive;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs{
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public Measure<Velocity<Angle>> yawVelocity = RadiansPerSecond.of(0);
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void deriveGyro(SwerveModuleState[] swerveModuleState, SwerveDriveKinematics kinematics) {}

    public default void resetHeading() {}
}