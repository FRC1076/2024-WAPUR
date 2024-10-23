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

    public static class TurretConstants {
        public static final int kMotorPort0 = 3; //PLACEHOLDER
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
}
