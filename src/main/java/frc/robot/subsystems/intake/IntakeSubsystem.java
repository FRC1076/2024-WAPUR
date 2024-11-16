package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public void runVolts(double volts) {
        this.io.runVolts(volts);
    }

    public void runVolts(Measure<Voltage> voltage) {
        runVolts(voltage.in(Volts));
    }

    public void stop() {
        runVolts(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake",inputs);
    }
}
