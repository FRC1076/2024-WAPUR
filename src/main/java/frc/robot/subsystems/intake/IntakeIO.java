package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Voltage;

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

    public default void runVolts(double volts) {}

    public default void runVolts(Measure<Voltage> voltage) {
        runVolts(voltage.in(Volts));
    }
}
