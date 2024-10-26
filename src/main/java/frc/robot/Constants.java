// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class Akit {

        /**
         * Determines the mode that AdvantageKit will run in.
         * <ul>
         * <li>0 = Running on a real robot</li>
         * <li>1 = Running on a simulator</li>
         * <li>2 = Replaying from a log file</li>
         * </ul>
         * currentMode's value can be changed as needed in the Constants.java file
         * before compile time. Please ensure that currentMode is set to 0 (real)
         * before pushing any changes to github.
         */
        public static final int currentMode = 0;
    }

    public static class turretConstants {
        public static final int kMotorPort0 = 3; //PLACEHOLDER
        public static class Control {
            public static class PIDCoefs {
                public static final double kP = 0.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
            }

            public static class PIDTolerance {
                public static final Measure<Angle> kPosition = Units.Degrees.of(5);
                public static final Measure<Velocity<Angle>> kVelocity = Units.DegreesPerSecond.of(3);
            }
        }
    }

    public static class driveConstants {

        public static class physical {
            public static final Measure<Distance> kWheelRadius = Units.Inches.of(6); //Placeholder
            public static final double kDriveGearRatio = 1.0; //Placeholder
            public static final double kTurnGearRatio = 1.0; //Placeholder
        }

        public static final class electrical {
            public static final Measure<Current> kDriveCurrentLimit = Units.Amps.of(40); // Current limit for drive motors
            public static final Measure<Current> kTurnCurrentLimit = Units.Amps.of(30);  // Current limit for turn motors
            public static final Measure<Voltage> kVoltageCompensation = Units.Volts.of(12); // Voltage compensation for swerve motors
        }

        public static class module0 {
            public static final int kDriveMotorPort = 0;
            public static final int kTurnMotorPort = 1;
            public static final int kAbsoluteEncoderPort = 2;
            public static final Rotation2d kAbsoluteEncoderOffset = new Rotation2d(Units.Degrees.of(5));
        }

        public static class module1 {
            public static final int kDriveMotorPort = 3;
            public static final int kTurnMotorPort = 4;
            public static final int kAbsoluteEncoderPort = 5;
            public static final Rotation2d kAbsoluteEncoderOffset = new Rotation2d(Units.Degrees.of(5));
        }

        public static class module2 {
            public static final int kDriveMotorPort = 6;
            public static final int kTurnMotorPort = 7;
            public static final int kAbsoluteEncoderPort = 8;
            public static final Rotation2d kAbsoluteEncoderOffset = new Rotation2d(Units.Degrees.of(5));
        }

        public static class module3 {
            public static final int kDriveMotorPort = 9;
            public static final int kTurnMotorPort = 10;
            public static final int kAbsoluteEncoderPort = 11;
            public static final Rotation2d kAbsoluteEncoderOffset = new Rotation2d(Units.Degrees.of(5));
        }
    }
}
