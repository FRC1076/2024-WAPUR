package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOHardware implements IntakeIO {

    private final CANSparkMax m_Motor;

    public IntakeIOHardware () {
        m_Motor = new CANSparkMax(IntakeConstants.kMotorPort, CANSparkMax.MotorType.kBrushless);
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