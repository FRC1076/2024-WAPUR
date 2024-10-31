package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOHardware implements IntakeIO {
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;

    public IntakeIOHardware () {
        m_leftMotor = new CANSparkMax(IntakeConstants.kLeftMotorPort, CANSparkMax.MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(IntakeConstants.kRightMotorPort, CANSparkMax.MotorType.kBrushless);
    }

    // no idea if this is correct
    @Override
    public void intakeOn() {
        m_leftMotor.set(IntakeConstants.kLeftMotorIntakeSpeed);
        m_rightMotor.set(IntakeConstants.kRightMotorIntakeSpeed);
    }

    @Override
    public void intakeOff() {
        m_leftMotor.set(0);
        m_rightMotor.set(0);
    }
}
