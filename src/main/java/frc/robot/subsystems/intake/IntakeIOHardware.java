package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.kCurrentLimit;
import static frc.robot.Constants.IntakeConstants.kMotorPort;
import static frc.robot.Constants.IntakeConstants.kVoltageCompensation;

public class IntakeIOHardware implements IntakeIO {

    private final CANSparkMax m_Motor;

    public IntakeIOHardware () {
        m_Motor = new CANSparkMax(kMotorPort, CANSparkMax.MotorType.kBrushless);
        m_Motor.setCANTimeout(250);
        m_Motor.enableVoltageCompensation(kVoltageCompensation.in(Volts));
        m_Motor.setSmartCurrentLimit((int) kCurrentLimit.in(Amps));
        m_Motor.setCANTimeout(0);
        m_Motor.burnFlash();
    }

    @Override
    public void runVolts(double volts) {
        m_Motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorAppliedVoltage = m_Motor.getAppliedOutput() * m_Motor.getBusVoltage();
        inputs.motorCurrent = m_Motor.getOutputCurrent();
    }
}