package frc.robot.subsystems.elevator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import com.revrobotics.CANSparkBase.ControlType;

import static frc.robot.Constants.ElevatorConstants.Control.kA;
import static frc.robot.Constants.ElevatorConstants.Control.kD;
import static frc.robot.Constants.ElevatorConstants.Control.kI;
import static frc.robot.Constants.ElevatorConstants.Control.kP;
import static frc.robot.Constants.ElevatorConstants.Control.kS;
import static frc.robot.Constants.ElevatorConstants.Control.kV;
import static frc.robot.Constants.ElevatorConstants.Control.kG;
import static frc.robot.Constants.ElevatorConstants.kMotorPort0;
import static frc.robot.Constants.ElevatorConstants.kMotorPort1;
import static frc.robot.Constants.ElevatorConstants.kPositionConversionFactor;
import static frc.robot.Constants.ElevatorConstants.kVelocityConversionFactor;
import static frc.robot.Constants.ElevatorConstants.Electrical.*;
import static edu.wpi.first.units.Units.*;

public class ElevatorIOHardware implements ElevatorIO {
    private CANSparkMax m_leadMotor; //Leader
    private CANSparkMax m_followMotor; //Follower
    
    private RelativeEncoder m_encoder;

    private SparkPIDController m_PIDController;

    private ElevatorFeedforward FFController = new ElevatorFeedforward(
        kS.in(Volts), 
        kG.in(Volts),
        kV.in(VoltsPerMeterPerSecond), 
        kA.in(VoltsPerMeterPerSecondSquared)
    );

    public ElevatorIOHardware() {
        m_leadMotor = new CANSparkMax(kMotorPort0,CANSparkMax.MotorType.kBrushless);
        m_followMotor = new CANSparkMax(kMotorPort1,CANSparkMax.MotorType.kBrushless);

        m_leadMotor.setCANTimeout(250);
        m_followMotor.setCANTimeout(250);

        m_leadMotor.restoreFactoryDefaults();
        m_followMotor.restoreFactoryDefaults();

        m_encoder = m_leadMotor.getEncoder();
        m_PIDController = m_leadMotor.getPIDController();

        m_PIDController.setP(kP);
        m_PIDController.setI(kI);
        m_PIDController.setD(kD);

        m_encoder.setPosition(0.0);

        m_encoder.setPositionConversionFactor(kPositionConversionFactor);
        m_encoder.setVelocityConversionFactor(kVelocityConversionFactor);
        
        m_encoder.setMeasurementPeriod(10);
        m_encoder.setAverageDepth(2);
        
        m_leadMotor.setSmartCurrentLimit((int) kCurrentLimit.in(Amps));
        m_followMotor.setSmartCurrentLimit((int) kCurrentLimit.in(Amps));

        m_leadMotor.enableVoltageCompensation(kVoltageCompensation.in(Volts));
        m_followMotor.enableVoltageCompensation(kVoltageCompensation.in(Volts));

        m_leadMotor.setCANTimeout(0);
        m_followMotor.setCANTimeout(0);
        m_leadMotor.burnFlash();
        m_followMotor.burnFlash();

        m_followMotor.follow(m_leadMotor);
        
    }

    @Override
    public void setVoltage(double volts){
        m_leadMotor.setVoltage(volts);
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond){
        m_PIDController.setReference(
            velocityMetersPerSecond,
            ControlType.kVelocity,
            0,
            FFController.calculate(velocityMetersPerSecond),
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVolts = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotor.getOutputCurrent();
        inputs.followCurrentAmps = m_followMotor.getOutputCurrent();
        inputs.elevatorHeightMeters = m_encoder.getPosition();
        inputs.velocityMetersPerSecond = m_encoder.getVelocity();
    }
}
