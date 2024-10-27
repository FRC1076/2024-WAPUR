// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
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
        public static class Driver{
            public static final int kControllerPort = 0;
            public static final double kControllerDeadband = 0.15;
            public static final double kControllerTriggerThreshold = 0.7;
        }
        public static class Operator{
            public static final int kControllerPort = 1;
            public static final double kControllerDeadband = 0.15;
            public static final double kControllerTriggerThreshold = 0.7;
        }
    }

    public static class Akit {

        /**
         * Determines the mode that AdvantageKit will run in.
         * <ul>
         * <li>Mode.REAL = Running on a real robot</li>
         * <li>Mode.SIM = Running on a simulator</li>
         * <li>Mode.REPLAY = Replaying from a log file</li>
         * </ul>
         * currentMode's value can be changed as needed in the Constants.java file
         * before compile time. Please ensure that currentMode is set to 0 (real)
         * before pushing any changes to github.
         */
        public static final Mode currentMode = Mode.REAL;

        public static enum Mode {
            REAL,
            SIM,
            REPLAY,
        }
    }

    public static class TurretConstants {
        public static final int kMotorPort0 = -1; //PLACEHOLDER
        public static class Control {
            public static class PIDCoefs {
                public static final double kP = 0.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
            }

            public static class PIDTolerance {
                public static final double kPosition = 0.0;
                public static final double kVelocity = 0.0;
            }
        }
    }

    public static class DriveConstants {

        public static class ModuleConstants {

            public static enum Corner {
                FrontLeft,
                FrontRight,
                RearRight,
                RearLeft
            }
            
            /** Constants that are common to all swerve modules */
            public static class Common {

                public static final Measure<Distance> kWheelDiameter = Inches.of(4);
                public static final Measure<Velocity<Distance>> maxSpeed = FeetPerSecond.of(3);
                public static final Measure<Voltage> kVoltageCompensation = Volts.of(12);

                public static class Drive {

                    public static final double gearRatio = 6.75;
                    public static final Measure<Velocity<Distance>> maxSpeed = FeetPerSecond.of(3);
                    public static final Measure<Current> kCurrentLimit = Amps.of(60);

                    public static class Control {
                        //PID Constants
                        public static final double kP = 0;
                        public static final double kI = 0;
                        public static final double kD = 0;

                        //Simple Motor FeedForward Constants
                        public static final double kS = 0;
                        public static final double kV = 2.78;
                        public static final double kA = 0;
                    }
                }

                public static class Turn {
                    public static final double gearRatio = 6.75;
                    public static final Measure<Current> kCurrentLimit = Amps.of(60);
                    public static final boolean turnMotorInverted = false;
                    public static final int kEncoderCPR = 4096;
                    public static final double kTurningEncoderDistancePerPulse =
                        // Assumes the encoders are on a 1:1 reduction with the module shaft.
                        (2 * Math.PI) / (double) kEncoderCPR;

                    public static class Control {
                        //PID Constants
                        public static final double kP = 0.4;
                        public static final double kI = 0;
                        public static final double kD = 0;
                        
                        //Feedforward Constants
                        public static final double kS = 0;
                        public static final double kV = 0;
                        public static final double kA = 0;
                        //Trapezoid Profile Constants
                        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 20 * Math.PI;
                        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 20 * Math.PI;
                    }
                }
            }

            public static class FrontLeftModule {
                public static final int kDriveMotorPort = 1;
                public static final int kTurnMotorPort = 11;
                public static final int kAbsoluteEncoderPort = 21;
                public static final Rotation2d kAbsoluteEncoderOffset = new Rotation2d(Degrees.of(77.9));
            }

            public static class FrontRightModule {
                public static final int kDriveMotorPort = 2;
                public static final int kTurnMotorPort = 12;
                public static final int kAbsoluteEncoderPort = 22;
                public static final Rotation2d kAbsoluteEncoderOffset = new Rotation2d(Degrees.of(-61.2));
            }

            public static class RearRightModule {
                public static final int kDriveMotorPort = 3;
                public static final int kTurnMotorPort = 13;
                public static final int kAbsoluteEncoderPort = 23;
                public static final Rotation2d kAbsoluteEncoderOffset = new Rotation2d(Degrees.of(113.5));
            }

            public static class RearLeftModule {
                public static final int kDriveMotorPort = 4;
                public static final int kTurnMotorPort = 14;
                public static final int kAbsoluteEncoderPort = 24;
                public static final Rotation2d kAbsoluteEncoderOffset = new Rotation2d(Degrees.of(125.9));
            }

            
        }
    }
}
