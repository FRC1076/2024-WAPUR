// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        
    }

    public static class ModuleConstants {
        public static final double kWheelDiameterMeters = 0.1016; // Four Inch Wheels
        public static class Drive {
            public static final double kDriveRatio = 6.75; // The ratio of a L2 SDS MK4 Module is 6.75 : 1
            // Also accounts for the gear ratio of the Swerve Module, L2 as of writing
            //Convert rotation to meters
            public static final double kDriveEncoderPositionConversionFactor = 
                (kWheelDiameterMeters * Math.PI) / kDriveRatio;
                
            //Convert RPM to m/s
            public static final double kDriveEncoderVelocityConversionFactor =
                kDriveEncoderPositionConversionFactor / 60;

            public static final double kPModuleDriveController = 0;
            public static final double kIModuleDriveController = 0;
            public static final double kDModuleDriveController = 0;

            public static final double ksDriveVolts = 0;
            public static final double kvDriveVoltSecondsPerMeter = 2.78;
            public static final double kaDriveVoltSecondsSquaredPerMeter = 0;

        }

        public static class Turning {
            public static final int kEncoderCPR = 4096;
            public static final double kTurningEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) kEncoderCPR;
            public static final double kPModuleTurningController = 0.4;
        }
     
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 20 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 20 * Math.PI;
        public static final double kMaxModuleSpeedMetersPerSecond = 3;
    }
}
