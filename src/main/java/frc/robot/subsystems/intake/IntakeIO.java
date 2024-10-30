package frc.robot.subsystems.intake;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double leftMotorVelocity = 0;
        public double leftMotorAppliedVoltage = 0;
        public double leftMotorCurrent = 0;

        public double rightMotorVelocity = 0;
        public double rightMotorAppliedVoltage = 0;
        public double rightMotorCurrent = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void intakeOn() {}

    public default void intakeOff() {}

    public default void intakeReversed() {}

    public default void setIntakeVelocity() {}
}
