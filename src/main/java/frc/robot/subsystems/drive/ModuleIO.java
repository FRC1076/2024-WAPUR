package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Velocity;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs{
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {}

    // ABOVE IS DYLAN'S WORK ON TRANSFERRING OVER REV SWERVE, BELOW IS FROM BEFORE

    /** Run the drive motor open-loop at the specified voltage. */
    public default void setDriveVoltage(double volts) {}

    /** Run the turn motor open-loop at the specified voltage. */
    public default void setTurnVoltage(double volts) {}

    public default void setDriveVoltage(Measure<Voltage> voltage) {
        setDriveVoltage(voltage.in(Volts));
    }

    public default void setTurnVoltage(Measure<Voltage> voltage) {
        setTurnVoltage(voltage.in(Volts));
    }

    /** Run the drive motor closed-loop at the specified velocity */
    public default void setDriveVelocity(double velocityRadiansPerSecond) {}

    /** Run the turn motor closed-loop to the specified position */
    public default void setTurnPosition(double positionRadians) {}

    public default void setDriveVelocity(Measure<Velocity<Angle>> velocity) {
        setDriveVelocity(velocity.in(RadiansPerSecond));
    }

    public default void setTurnPosition(Measure<Angle> position) {
        setTurnPosition(position.in(Radians));
    }

    /** Enable or disable brake mode on the drive motor. */
    public default void setDriveBrakeMode(boolean enable) {}

    /** Enable or disable brake mode on the turn motor. */
    public default void setTurnBrakeMode(boolean enable) {}
}