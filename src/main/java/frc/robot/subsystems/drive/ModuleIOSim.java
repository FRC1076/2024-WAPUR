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
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.littletonrobotics.junction.Logger;;


/* IO for a Spark Max operating in brushless mode */
public class ModuleIOSim implements ModuleIO {
    
    //Need to implement constants
    // private final DCMotorSim m_driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    // private final DCMotorSim m_turnSim = new DCMotorSim(DCMotor.getNEO(1), 12.8, 0.004);
    private final DCMotorSim m_driveSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.01);
    private final DCMotorSim m_turnSim = new DCMotorSim(DCMotor.getNEO(1), 12.8, 0.004);
    private double turnSetpoint = 0;
    private double driveSetpoint = 0;
    private double driveVoltage = 0;
    private double turnVoltage = 0;

    private final PIDController m_turnPIDController = new PIDController(0.4, 0, 0);

    private final PIDController m_drivePIDController = new PIDController(2.4, 0, 0);
    private final SimpleMotorFeedforward m_driveFFController = new SimpleMotorFeedforward(0, 2.78, 0);
    

    // Take in parameters to specify exact module
    public ModuleIOSim(){
        m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs){

        // Update the sim loop
        m_driveSim.update(0.02);
        m_turnSim.update(0.02);

        inputs.drivePositionMeters = m_driveSim.getAngularPositionRotations() * Common.Drive.positionConversionFactor.in(Meter);
        inputs.driveVelocityMetersPerSec = m_driveSim.getAngularVelocityRPM() * Common.Drive.velocityConversionFactor.in(MetersPerSecond);

        //double wrappedTurnPosition = m_turnSim.getAngularPositionRad()

        inputs.turnPosition = Rotation2d.fromRadians(m_turnSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();
        inputs.turnAbsolutePosition = Rotation2d.fromRadians(m_turnSim.getAngularPositionRad());
    
        inputs.driveAppliedVolts = driveVoltage;
        inputs.turnAppliedVolts = turnVoltage;

        inputs.driveCurrentAmps = m_driveSim.getCurrentDrawAmps();
        inputs.turnCurrentAmps = m_turnSim.getCurrentDrawAmps();

        inputs.turnSetpoint = turnSetpoint;
        inputs.driveSetpoint = driveSetpoint;
    }

    /** 
     * Periodic Tasks:
     * 
     * -Update Status Signal
     * */
    @Override
    public void periodic(){
        
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveVoltage = volts;
        m_driveSim.setInputVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnVoltage = volts;
        m_turnSim.setInputVoltage(volts);
    }

    @Override 
    public void setDriveVelocity(double velocityMetersPerSecond) {
        driveSetpoint = velocityMetersPerSecond;
        setDriveVoltage(
            m_drivePIDController.calculate(
                m_driveSim.getAngularVelocityRPM() * Common.Drive.velocityConversionFactor.in(MetersPerSecond), 
                velocityMetersPerSecond) 
                + m_driveFFController.calculate(velocityMetersPerSecond)
        );
    }

    @Override
    public void setTurnPosition(double positionRadians) {
        turnSetpoint = positionRadians;
        double wrappedPosition = m_turnSim.getAngularPositionRad() % (2 * Math.PI);
        if(Math.abs(wrappedPosition) > Math.PI) wrappedPosition = ((2 * Math.PI) - Math.abs(wrappedPosition)) * Math.signum(-wrappedPosition); 
        Logger.recordOutput("TEST", wrappedPosition);
        setTurnVoltage(m_turnPIDController.calculate(m_turnSim.getAngularPositionRad(), positionRadians));
    }

    @Override
    public void setDriveBrakeMode(boolean enable){
        return;
    }

    @Override
    public void setTurnBrakeMode(boolean enable){
        return;
    }

    
    @Override
    public void updateTurnEncoder() {
        return;
    }
    
}
