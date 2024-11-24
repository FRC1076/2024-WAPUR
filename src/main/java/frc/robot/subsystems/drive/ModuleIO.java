package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs{
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveSetpoint = 0.0;

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnSetpoint = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Any miscellaneous periodic routines that need to be run */
    public default void periodic() {}

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

    public default void setTurnPosition(Rotation2d rot) {
        setTurnPosition(rot.getRadians());
    }

    /** Enable or disable brake mode on the drive motor. */
    public default void setDriveBrakeMode(boolean enable) {}

    /** Enable or disable brake mode on the turn motor. */
    public default void setTurnBrakeMode(boolean enable) {}

    /** Updates relative turn encoder with value from absolute encoder */
    public default void updateTurnEncoder() {}
}