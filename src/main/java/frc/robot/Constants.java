// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static frc.robot.utils.units.Units.RadiansPerSecondSquared;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class GameConstants {
        public static final String kTeamColorBlue = "BLUE";
        public static final String kTeamColorRed = "RED";
        public static String teamColor = kTeamColorBlue; // "BLUE", "RED"

        public static final String kStartLeftSide = "LEFT";
        public static final String kStartRightSide = "RIGHT";
        public static String startPosition = kStartLeftSide; // "LEFT", "RIGHT"
    }
    public static class OIConstants {
        public static class Driver{
            public static final int kControllerPort = 0;
            public static final double kControllerDeadband = 0.15;
            public static final double kControllerTriggerThreshold = 0.7;
            public static final double kRotClutchFactor = 0.5;
        }
        public static class Operator{
            public static final int kControllerPort = 1;
            public static final double kControllerDeadband = 0.15;
            public static final double kControllerTriggerThreshold = 0.7;
            public static final double kElevatorManualSpeedLimit = 0.5; //Speed limit for elevator manual velocity control (m/s)
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

    public static class DriveConstants {
        public static Measure<Velocity<Distance>> kMaxTransSpeed = FeetPerSecond.of(5); // This is the maximum translational speed allowed by the rules
        public static Measure<Velocity<Angle>> kMaxRotSpeed = RotationsPerSecond.of(1); // This is the maximum rotational speed allowed by the rules
        
        public static double kSingleClutchTranslation = 0.5;
        public static double kSingleClutchRotation = 0.5;

        public static double kDoubleClutchTranslation = 0.3;
        public static double kDoubleClutchRotation = 0.35;

        public static class Physical {
            public static Measure<Distance> kWheelBase = Inches.of(27.5);
            public static Measure<Distance> kTrackWidth= Inches.of(19.5);
            public static Measure<Distance> kDriveBaseRadius = Inches.of(Math.hypot(kWheelBase.in(Inches), kTrackWidth.in(Inches)));
        }

        public static class ModuleConstants {
            public static enum Corner {
                FrontLeft("FrontLeftModule"),
                FrontRight("FrontRightModule"),
                RearRight("RearRightModule"),
                RearLeft("RearLeftModule");

                public final String label;

                private Corner(String label){
                    this.label = label;
                }
            }
            
            /** Constants that are common to all swerve modules */
            public static class Common {
                public static final Measure<Distance> kWheelDiameter = Meter.of(0.1016);
                public static final Measure<Velocity<Distance>> kMaxModuleSpeed = FeetPerSecond.of(14); // This is the top speed a module is physically capable of reaching
                public static final Measure<Voltage> kVoltageCompensation = Volts.of(12);
                public static final Measure<Voltage> kMaxDriveVolts = Volts.of(12); // Maximum applied voltage in openloop driving

                public static class Drive {

                    public static final double gearRatio = 6.75;
                    public static final Measure<Distance> positionConversionFactor = Meter.of((1/gearRatio) * Inches.of(4*Math.PI).in(Meter)); //gear ratio, circumference of the wheel, in -> m
                    public static final Measure<Velocity<Distance>> velocityConversionFactor = MetersPerSecond.of((1/gearRatio) * Inches.of(4*Math.PI).in(Meter) / 60); //gear ratio, circumference of the wheel, in -> m, mintues -> seconds
                    public static final Measure<Current> kCurrentLimit = Amps.of(60);

                    public static class Control {
                        //PID Constants
                        public static final double kP = 0.2;
                        public static final double kI = 0;
                        public static final double kD = 0.012;

                        //Simple Motor FeedForward Constants
                        public static final double kS = 0;
                        public static final double kV = 2.78;
                        public static final double kA = 0;
                    }
                }

                public static class Turn {
                    public static final double gearRatio = 12.8;
                    public static final Measure<Angle> positionConversionFactor = Radians.of((1/12.8) * (2*Math.PI));
                    public static final Measure<Velocity<Angle>> velocityConversionFactor = RadiansPerSecond.of((1/12.8) * (2*Math.PI) * 60);
                    public static final Measure<Current> kCurrentLimit = Amps.of(60);
                    public static final boolean turnMotorInverted = false;
                    public static final int kEncoderCPR = 4096;
                    public static final double kTurningEncoderDistancePerPulse =
                        //Assumes the encoders are on a 1:1 reduction with the module shaft.
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
                        public static final Measure<Velocity<Angle>> kMaxAngularSpeed = RadiansPerSecond.of(20 * Math.PI);
                        public static final Measure<Velocity<Velocity<Angle>>> kMaxAngularAccel = RadiansPerSecondSquared.of(20 * Math.PI);
                    }
                }
            }

            public static class FrontLeftModule {
                public static final int kDriveMotorPort = 1; //CAN ID Updated
                public static final int kTurnMotorPort = 11; //CAN ID Updated
                public static final int kAbsoluteEncoderPort = 21;
                public static final Rotation2d kAbsoluteEncoderOffset = new Rotation2d(Rotations.of(-0.441162109375 +0.5));//new Rotation2d(Rotations.of(-0.429931640625 + 0.5)); //2.295 - Math.PI/2
                public static final boolean invertDriveMotor = false;
            }

            public static class FrontRightModule {
                public static final int kDriveMotorPort = 2; //CAN ID updated
                public static final int kTurnMotorPort = 12; //CAN ID updated
                public static final int kAbsoluteEncoderPort = 22;
                public static final Rotation2d kAbsoluteEncoderOffset = new Rotation2d(Rotations.of(-0.3984375 +0.5));//new Rotation2d(Rotations.of(-0.403564453125 + 0.5)); //0.781
                public static final boolean invertDriveMotor = false;
            }

            public static class RearRightModule {
                public static final int kDriveMotorPort = 3; //CAN ID updated
                public static final int kTurnMotorPort = 13; //CAN ID updated
                public static final int kAbsoluteEncoderPort = 23; 
                public static final Rotation2d kAbsoluteEncoderOffset = new Rotation2d(Rotations.of(-0.525146484375 +0.5));//new Rotation2d(Rotations.of(0.4765625 + 0.5)); //5.134
                public static final boolean invertDriveMotor = false;
            }

            public static class RearLeftModule {
                public static final int kDriveMotorPort = 4; //CAN ID Updated
                public static final int kTurnMotorPort = 14; //CAN ID Updated
                public static final int kAbsoluteEncoderPort = 24;
                public static final Rotation2d kAbsoluteEncoderOffset = new Rotation2d(Rotations.of(-0.931396484375 +0.5));//new Rotation2d(Rotations.of(0.078369140625 + 0.5)); //3.863 - Math.PI/2
                public static final boolean invertDriveMotor = false;
            }    
        }

        public static class GyroConstants {
            
            public static final int kGyroPort = 9;
            
            public static class Signal {
                public static final double kYawUpdateFrequencyHz = 100;
                public static final double kYawVelocityUpdateFrequencyHz = 100;
            }
        }
    }

    public static class IntakeConstants {
        public static final int kMotorPort = 51; //CAN ID updated

        public static final Measure<Current> kCurrentLimit = Amps.of(40);
        public static final Measure<Voltage> kVoltageCompensation = Volts.of(12);
    }

    public static class GrabberConstants {
        public static final int kMotorPort = 33; //CAN ID updated
        
        public static final Measure<Current> kCurrentLimit = Amps.of(40); 
        public static final Measure<Voltage> kVoltageCompensation = Volts.of(12);
    }

    public static class ShooterConstants {
        public static final int kMotorPort = 52; //CAN ID Updated
        public static final int kServoPort = 1; //Connected directly to RIO

        public static final Measure<Current> kCurrentLimit = Amps.of(40); 
        public static final Measure<Voltage> kVoltageCompensation = Volts.of(12);
    }

    public static class ElevatorConstants {
        public static final int kMotorPort0 = 31; //CAN ID updated
        public static final int kMotorPort1 = 32; //CAN ID updated

        public static final boolean leadMotorInverted = false;
        public static final boolean followMotorInverted = false;

        //Heights measured in meters
        public static final double lowHeight = 0;
        public static final double autonHeight = 0.4;
        public static final double midHeight = 0.42;
        public static final double highHeight = 0.84;

        //public static final double minHeightMeters = 0;
        //public static final double maxHeightMeters = 0.85; //Temporary

        //htt;s://wcproducts.com/collections/gearboxes/products/wcp-single-stage-gearbox  Inches.of(0.25).in(Meters)
        public static final double kVelocityConversionFactor = (11/60.0) * 22 * 0.00635 / 60.0; //Gear ratio & chain pitch & rpm -> m/s
        public static final double kPositionConversionFactor = (11/60.0) * 22 * 0.00635; //Gear ratio & chain pitch
        public static class Electrical {
            public static final Measure<Voltage> kVoltageCompensation = Volts.of(12);
            public static final Measure<Current> kCurrentLimit = Amps.of(40);
        }

        public static class Control {
            //PID Constants
            public static final double kP = 4;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            //Feedforward Constants
            public static final Measure<Voltage> kS = Volts.of(0); //Static gain (voltage)
            public static final Measure<Voltage> kG = Volts.of(0.6); //Gravity gain (voltage)
            public static final Measure<Per<Voltage,Velocity<Distance>>> kV = VoltsPerMeterPerSecond.of(12); //Velocity Gain
            public static final Measure<Per<Voltage,Velocity<Velocity<Distance>>>> kA = VoltsPerMeterPerSecondSquared.of(0); //Acceleration Gain
        }

        public static class PositionControl {
            public static final double kP = 0.4;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }
    }

    public static class AutoConstants {
        public static class Control {
            public static class Translation {
                public static final double kP = 5;
                public static final double kI = 0;
                public static final double kD = 0;
            }
            public static class Rotation {
                public static final double kP = 5;
                public static final double kI = 0;
                public static final double kD = 0;
            }
        }
    }
}
