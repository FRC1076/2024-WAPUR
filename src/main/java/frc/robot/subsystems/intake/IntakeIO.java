package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Voltage;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double motorAppliedVoltage = 0;
        public double motorCurrent = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default void runVolts(Measure<Voltage> voltage) {
        runVolts(voltage.in(Volts));
    }
}
