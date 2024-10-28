package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.units.Units.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModuleConstants.FrontLeftModule;
import frc.robot.Constants.DriveConstants.ModuleConstants.FrontRightModule;
import frc.robot.Constants.DriveConstants.ModuleConstants.RearRightModule;
import frc.robot.Constants.DriveConstants.ModuleConstants.RearLeftModule;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.Constants.DriveConstants.ModuleConstants.Common;


/* IO for a Spark Max operating in brushless mode */
public class ModuleIOHardware implements ModuleIO {
    
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turnMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turnRelativeEncoder;
    private final CANcoder m_turnAbsoluteEncoder;

    private final StatusSignal<Double> turnAbsolutePosition;

    private final SparkPIDController m_drivePIDController;
    private final SparkPIDController m_turnPIDController;

    private final Rotation2d absoluteEncoderOffset;

    // Units should be in radians/sec
    private final SimpleMotorFeedforward m_driveFFController = 
        new SimpleMotorFeedforward(
            Common.Drive.Control.kS,
            Common.Drive.Control.kV,
            Common.Drive.Control.kA);
    
    // Units should be in radians
    private final SimpleMotorFeedforward m_turnFFController = 
        new SimpleMotorFeedforward(
            Common.Turn.Control.kS,
            Common.Turn.Control.kV,
            Common.Turn.Control.kA);
    

    // Take in parameters to specify exact module
    public ModuleIOHardware(ModuleConstants.Corner corner){
        switch (corner) {
            case FrontLeft -> {
                m_driveMotor = new CANSparkMax(FrontLeftModule.kDriveMotorPort,CANSparkMax.MotorType.kBrushless);
                m_turnMotor = new CANSparkMax(FrontLeftModule.kTurnMotorPort,CANSparkMax.MotorType.kBrushless);
                m_turnAbsoluteEncoder = new CANcoder(FrontLeftModule.kAbsoluteEncoderPort);
                absoluteEncoderOffset = FrontLeftModule.kAbsoluteEncoderOffset;
            }
            case FrontRight -> {
                m_driveMotor = new CANSparkMax(FrontRightModule.kDriveMotorPort,CANSparkMax.MotorType.kBrushless);
                m_turnMotor = new CANSparkMax(FrontRightModule.kTurnMotorPort,CANSparkMax.MotorType.kBrushless);
                m_turnAbsoluteEncoder = new CANcoder(FrontRightModule.kAbsoluteEncoderPort);
                absoluteEncoderOffset = FrontRightModule.kAbsoluteEncoderOffset;
            }
            case RearRight -> {
                m_driveMotor = new CANSparkMax(RearRightModule.kDriveMotorPort,CANSparkMax.MotorType.kBrushless);
                m_turnMotor = new CANSparkMax(RearRightModule.kTurnMotorPort,CANSparkMax.MotorType.kBrushless);
                m_turnAbsoluteEncoder = new CANcoder(RearRightModule.kAbsoluteEncoderPort);
                absoluteEncoderOffset = RearRightModule.kAbsoluteEncoderOffset;
            }
            case RearLeft -> {
                m_driveMotor = new CANSparkMax(RearLeftModule.kDriveMotorPort,CANSparkMax.MotorType.kBrushless);
                m_turnMotor = new CANSparkMax(RearLeftModule.kTurnMotorPort,CANSparkMax.MotorType.kBrushless);
                m_turnAbsoluteEncoder = new CANcoder(RearLeftModule.kAbsoluteEncoderPort);
                absoluteEncoderOffset = RearLeftModule.kAbsoluteEncoderOffset;
            }
            default -> throw new RuntimeException("Invalid module index");
        }

        CANcoderConfiguration configs = new CANcoderConfiguration();
        configs.FutureProofConfigs = false;
        configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        configs.MagnetSensor.MagnetOffset = absoluteEncoderOffset.getRotations();
        configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        m_turnAbsoluteEncoder.getConfigurator().apply(configs);
        
        turnAbsolutePosition = m_turnAbsoluteEncoder.getAbsolutePosition();

        m_driveMotor.restoreFactoryDefaults();
        m_turnMotor.restoreFactoryDefaults();

        m_driveMotor.setCANTimeout(250);
        m_turnMotor.setCANTimeout(250);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnRelativeEncoder = m_turnMotor.getEncoder();

        m_drivePIDController = m_driveMotor.getPIDController();
        m_turnPIDController = m_turnMotor.getPIDController();

        m_drivePIDController.setP(Common.Drive.Control.kP);
        m_drivePIDController.setI(Common.Drive.Control.kI);
        m_drivePIDController.setD(Common.Drive.Control.kD);

        m_turnPIDController.setP(Common.Turn.Control.kP);
        m_turnPIDController.setI(Common.Turn.Control.kI);
        m_turnPIDController.setD(Common.Turn.Control.kD);

        m_turnMotor.setInverted(Common.Turn.turnMotorInverted);
        m_driveMotor.setSmartCurrentLimit((int) Common.Drive.kCurrentLimit.in(Amps));
        m_turnMotor.setSmartCurrentLimit((int) Common.Turn.kCurrentLimit.in(Amps));

        m_driveEncoder.setPosition(0.0);
        m_driveEncoder.setMeasurementPeriod(10);
        m_driveEncoder.setAverageDepth(2);

        m_turnRelativeEncoder.setPosition(turnAbsolutePosition.getValueAsDouble());
        m_turnRelativeEncoder.setMeasurementPeriod(10);
        m_turnRelativeEncoder.setAverageDepth(2);

        m_driveMotor.setCANTimeout(0);
        m_turnMotor.setCANTimeout(0);

        m_driveMotor.burnFlash();
        m_turnMotor.burnFlash();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs){

        inputs.drivePositionRad = 
        Units.rotationsToRadians(m_driveEncoder.getPosition()/Common.Drive.gearRatio);
        inputs.driveVelocityRadPerSec = 
            Units.rotationsPerMinuteToRadiansPerSecond(m_driveEncoder.getVelocity()/Common.Drive.gearRatio);

        inputs.turnPosition = 
            Rotation2d.fromRotations(m_turnRelativeEncoder.getPosition()/Common.Turn.gearRatio);
        inputs.turnVelocityRadPerSec = 
            Units.rotationsPerMinuteToRadiansPerSecond(m_turnRelativeEncoder.getVelocity()/Common.Drive.gearRatio);
        inputs.turnAbsolutePosition =
            Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    
        inputs.driveAppliedVolts = m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();
        inputs.turnAppliedVolts = m_turnMotor.getAppliedOutput() * m_turnMotor.getBusVoltage();

        inputs.driveCurrentAmps = m_driveMotor.getOutputCurrent();
        inputs.turnCurrentAmps = m_turnMotor.getOutputCurrent();

    }

    /** 
     * Periodic Tasks:
     * 
     * -
     * */
    @Override
    public void periodic(){}

    @Override
    public void setDriveVoltage(double volts) {
        m_driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        m_turnMotor.setVoltage(volts);
    }

    @Override 
    public void setDriveVelocity(double velocityRadiansPerSecond) {
        m_drivePIDController.setReference(
            Units.radiansPerSecondToRotationsPerMinute(velocityRadiansPerSecond),
            ControlType.kVelocity,
            0,
            m_driveFFController.calculate(velocityRadiansPerSecond),
            ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(double positionRadians) {
        m_turnPIDController.setReference(
            Units.radiansToRotations(positionRadians),
            ControlType.kPosition,
            0,
            m_turnFFController.calculate(positionRadians),
            ArbFFUnits.kVoltage);
    }

    @Override
    public void setDriveBrakeMode(boolean enable){
        m_driveMotor.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void setTurnBrakeMode(boolean enable){
        m_turnMotor.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

}
