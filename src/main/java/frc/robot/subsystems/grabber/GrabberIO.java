package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Voltage;

public interface GrabberIO {
    @AutoLog
    public static class GrabberIOInputs {
        public double motorAppliedVoltage = 0;
        public double motorCurrent = 0;
    }

    public default void updateInputs(GrabberIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default void runVolts(Measure<Voltage> voltage) {
        runVolts(voltage.in(Volts));
    }
}
