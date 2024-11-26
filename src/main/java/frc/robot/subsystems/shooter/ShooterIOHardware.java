package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOHardware implements ShooterIO {
    private final CANSparkMax m_Motor;

    public ShooterIOHardware() {
        m_Motor = new CANSparkMax(ShooterConstants.kMotorPort, CANSparkMax.MotorType.kBrushless);
    }
    
    @Override
    public void runVolts(double Volts) {
        m_Motor.setVoltage(Volts);

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorAppliedVoltage = m_Motor.getAppliedOutput() * m_Motor.getBusVoltage();
        inputs.motorCurrent = m_Motor.getOutputCurrent();
    }
}