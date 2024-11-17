package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.driveConstants;
/**
 * IO Layer for a CAN Spark Max with a CANCoder
 */
public class ModuleIOSparkMaxCC implements ModuleIOBase {

    private static final double DRIVE_GEAR_RATIO = driveConstants.physical.kDriveGearRatio;
    private static final double TURN_GEAR_RATIO = driveConstants.physical.kTurnGearRatio;

    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;
    
    private final RelativeEncoder driveRelativeEncoder;
    private final RelativeEncoder turnRelativeEncoder;
    private final CANcoder turnAbsoluteEncoder;
    private final StatusSignal<Double> turnAbsolutePosition;

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOSparkMaxCC(int index) {
        switch (index) {
            case 0 -> {
                driveSparkMax = new CANSparkMax(driveConstants.module0.kDriveMotorPort,CANSparkMax.MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(driveConstants.module0.kTurnMotorPort,CANSparkMax.MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(driveConstants.module0.kAbsoluteEncoderPort);
                absoluteEncoderOffset = driveConstants.module0.kAbsoluteEncoderOffset;
            }
            case 1 -> {
                driveSparkMax = new CANSparkMax(driveConstants.module1.kDriveMotorPort,CANSparkMax.MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(driveConstants.module1.kTurnMotorPort,CANSparkMax.MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(driveConstants.module1.kAbsoluteEncoderPort);
                absoluteEncoderOffset = driveConstants.module1.kAbsoluteEncoderOffset;
            }
            case 2 -> {
                driveSparkMax = new CANSparkMax(driveConstants.module2.kDriveMotorPort,CANSparkMax.MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(driveConstants.module2.kTurnMotorPort,CANSparkMax.MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(driveConstants.module2.kAbsoluteEncoderPort);
                absoluteEncoderOffset = driveConstants.module2.kAbsoluteEncoderOffset;
            }
            case 3 -> {
                driveSparkMax = new CANSparkMax(driveConstants.module0.kDriveMotorPort,CANSparkMax.MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(driveConstants.module0.kTurnMotorPort,CANSparkMax.MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(driveConstants.module0.kAbsoluteEncoderPort);
                absoluteEncoderOffset = driveConstants.module3.kAbsoluteEncoderOffset;
            }
            default -> throw new RuntimeException("Invalid module index");
        }

        turnAbsolutePosition = turnAbsoluteEncoder.getAbsolutePosition();

        driveSparkMax.restoreFactoryDefaults();
        turnSparkMax.restoreFactoryDefaults();

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        driveRelativeEncoder = driveSparkMax.getEncoder();
        turnRelativeEncoder = turnSparkMax.getEncoder();

        turnSparkMax.setInverted(isTurnMotorInverted);
        driveSparkMax.setSmartCurrentLimit((int) driveConstants.electrical.kDriveCurrentLimit.in(edu.wpi.first.units.Units.Amps));
        turnSparkMax.setSmartCurrentLimit((int) driveConstants.electrical.kTurnCurrentLimit.in(edu.wpi.first.units.Units.Amps));
        driveSparkMax.enableVoltageCompensation(driveConstants.electrical.kVoltageCompensation.in(edu.wpi.first.units.Units.Volts));
        turnSparkMax.enableVoltageCompensation(driveConstants.electrical.kVoltageCompensation.in(edu.wpi.first.units.Units.Volts));

        driveRelativeEncoder.setPosition(0.0);
        driveRelativeEncoder.setMeasurementPeriod(10);
        driveRelativeEncoder.setAverageDepth(2);

        turnRelativeEncoder.setPosition(0.0);
        turnRelativeEncoder.setMeasurementPeriod(10);
        turnRelativeEncoder.setAverageDepth(2);

        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

        driveSparkMax.burnFlash();
        turnSparkMax.burnFlash();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        inputs.drivePositionRad = 
            Units.rotationsToRadians(driveRelativeEncoder.getPosition()/DRIVE_GEAR_RATIO);
        inputs.driveVelocityRadPerSec = 
            Units.rotationsPerMinuteToRadiansPerSecond(driveRelativeEncoder.getVelocity()/DRIVE_GEAR_RATIO);

        inputs.turnPosition = 
            Rotation2d.fromRotations(turnRelativeEncoder.getPosition()/TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec = 
            Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()/TURN_GEAR_RATIO);
        inputs.turnAbsolutePosition =
            Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
                .minus(absoluteEncoderOffset);
        
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();

        inputs.driveCurrentAmps = driveSparkMax.getOutputCurrent();
        inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSparkMax.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable){
        driveSparkMax.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void setTurnBrakeMode(boolean enable){
        turnSparkMax.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
}
