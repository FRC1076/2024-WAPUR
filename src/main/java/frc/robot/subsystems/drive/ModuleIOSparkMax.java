package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.drive.ModuleIOBase;
import frc.robot.subsystems.drive.ModuleIOBase.ModuleIOInputs;

/* IO for a Spark Max operating in brushless mode */
public class ModuleIOSparkMax implements ModuleIOBase {
    
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final CANCoder m_turningEncoder;

    private final SparkPIDController m_drivePIDController;
    
    /*private final SimpleMotorFeedforward m_driveFFController = 
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
                ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));*/

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

    }
}
