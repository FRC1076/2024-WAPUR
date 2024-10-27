package frc.robot.utils.units;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.DriveConstants.ModuleConstants.Common.Drive.gearRatio;
import static frc.robot.Constants.DriveConstants.ModuleConstants.Common.kWheelDiameter;
/**
 * provides helper functions for unit conversions that are not supported by the WPILib Units library
 * (e.g. conversions from one dimension type into another)
 */
public final class UnitConversion {
    private UnitConversion() {} // Private constructor prevents instantiation
    public static Measure<Distance> EncoderPositionToLinearDistance(Measure<Angle> encoderPosition) {
        return Meters.of((encoderPosition.in(Radians) * kWheelDiameter.in(Meters) * Math.PI)/gearRatio);
    }
}
