package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;

import frc.robot.Constants.turretConstants;

/* IO for a Spark Max operating in brushless mode */
public class TurretIOSparkMax implements TurretIOBase {
    private final CANSparkMax m_motor0 = new CANSparkMax(turretConstants.kMotorPort0,MotorType.kBrushless);
    private final RelativeEncoder m_encoder0 = m_motor0.getEncoder();
    private final SparkPIDController m_controller = m_motor0.getPIDController();
    //TODO: Tune PID
    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;

    public TurretIOSparkMax(){
        m_motor0.enableVoltageCompensation(12.0);

        m_controller.setP(kP);
        m_controller.setI(kI);
        m_controller.setD(kD);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs){
        inputs.turretPositionRad = Units.rotationsToRadians(m_encoder0.getPosition());
        inputs.turretPositionRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(m_encoder0.getVelocity());

        inputs.turretAppliedVolts = m_motor0.getAppliedOutput() * m_motor0.getBusVoltage(); //the SparkMax API somehow doesn't have a built-in method to get output voltage
        inputs.turretCurrentAmps = new double[] {m_motor0.getOutputCurrent()};
    }

    @Override
    public void setVoltage(double turretVolts) {
        m_motor0.setVoltage(turretVolts);
    }

    @Override
    public void setPositionRad(double turretPositionRads, double turretFFVolts) {
        m_controller.setReference(
            Units.radiansToRotations(turretPositionRads),
            ControlType.kPosition,
            0,
            turretFFVolts
        );
    }
}
