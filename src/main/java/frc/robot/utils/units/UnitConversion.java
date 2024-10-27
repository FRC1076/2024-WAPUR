package frc.robot.utils.units;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.ModuleConstants.Common.Drive.gearRatio;
import static frc.robot.Constants.DriveConstants.ModuleConstants.Common.kWheelDiameter;
/**
 * provides helper functions for unit conversions that are not supported by the WPILib Units library
 * (e.g. conversions from one dimension type into another)
 */
public final class UnitConversion {
    private UnitConversion() {} // Private constructor prevents instantiation
    public static Measure<Distance> EncoderPositionToLinearDistance(Measure<Angle> encoderPosition) {
        return Meters.of((encoderPosition.in(Radians) * kWheelDiameter.in(Meters))/gearRatio);
    }

    public static Measure<Velocity<Distance>> EncoderVelocityToLinearVelocity(Measure<Velocity<Angle>> encoderVelocity) {
        return MetersPerSecond.of((encoderVelocity.in(RadiansPerSecond) * kWheelDiameter.in(Meters))/gearRatio);
    }

    public static Measure<Velocity<Angle>> LinearVelocityToAngularVelocity(Measure<Velocity<Distance>> linearVelocity) {
        return RadiansPerSecond.of((linearVelocity.in(MetersPerSecond)/kWheelDiameter.in(Meters)) * gearRatio);
    }

}
