package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.util.Units;

public class TurretSubsystem extends SubsystemBase {
    private final TurretIOBase io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    
    public TurretSubsystem(TurretIOBase io){
        this.io = io;
    }

    public void turnVolts(double volts){
        io.setVoltage(volts);
    }

    public void setPositionRad(double posRad){
        io.setPositionRad(posRad,0.0);
    }

    public void setPositionDeg(double posDeg){
        io.setPositionRad(Units.degreesToRadians(posDeg),0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive",inputs);
    }
}