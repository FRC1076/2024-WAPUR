package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.drive.ModuleIOBase;
import frc.robot.subsystems.drive.ModuleIOBase.ModuleIOInputs;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;

/* IO for a Spark Max operating in brushless mode */
public class ModuleIOSparkMax implements ModuleIOBase {
    
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final CANCoder m_turningEncoder;

    private final SparkPIDController m_drivePIDController;
    
    private final SimpleMotorFeedforward m_driveFFController = 
        new SimpleMotorFeedforward(
            ModuleConstants.ksDriveVolts,
            ModuleConstants.kvDriveVoltSecondsPerMeter,
            ModuleConstants.kaDriveVoltSecondsSquaredPerMeter);

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController m_turningPIDController = // new PIDController(ModuleConstants.kPModuleTurningController, 0, 0);
    
        new ProfiledPIDController(
            ModuleConstants.kPModuleTurningController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    // Take in parameters to specify exact module
    public ModuleIOSparkMax(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel){
        m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        m_drivePIDController = m_driveMotor.getPIDController();
        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
        m_turningEncoder = new CANCoder(turningEncoderChannel);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs){
        
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        Rotation2d encoderRotation = new Rotation2d(m_turningEncoder.getAbsolutePosition());

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

        if(!isOpenLoop){
        //In closed loop drive, use on board pid controller for drive motor
        m_drivePIDController.setReference(
            state.speedMetersPerSecond,
            ControlType.kVelocity,
            0,
            m_driveFFController.calculate(state.speedMetersPerSecond),
            ArbFFUnits.kVoltage);
        }
        else{
            // Divide the drive output by the max speed to scale it from -1 to 1 and make it open loop
            m_driveMotor.set(state.speedMetersPerSecond / ModuleConstants.kMaxModuleSpeedMetersPerSecond);
        }

        // Calculate the turning motor output from the turning PID controller.
        m_turningMotor.set(
            m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition(), state.angle.getRadians())
        );
    }
}
