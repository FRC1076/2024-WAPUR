package frc.robot.subsystems.grabber;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.GrabberConstants;

public class GrabberIOHardware implements GrabberIO{
    private final CANSparkMax m_Motor;

    public GrabberIOHardware() {
        //motor port constant is currently unknown. Change when known.
        m_Motor = new CANSparkMax(GrabberConstants.kMotorPort, CANSparkMax.MotorType.kBrushless);
    }

    @Override
    public void runVolts(double volts) {
        m_Motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.motorAppliedVoltage = m_Motor.getAppliedOutput() * m_Motor.getBusVoltage();
        inputs.motorCurrent = m_Motor.getOutputCurrent();
    }
}