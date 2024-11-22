package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Twist2d;
import org.littletonrobotics.junction.Logger;

public class GyroIOSim implements GyroIO {

    private Rotation2d simRotation = new Rotation2d();

    public GyroIOSim(){}

    @Override
    public void updateInputs(GyroIOInputs inputs){
        inputs.connected = true;
        inputs.yawPosition = simRotation;
        inputs.yawVelocity = RadiansPerSecond.of(0);
    }

    //Must be called periodically
    @Override
    public void deriveGyro(SwerveModulePosition[] swerveModuleDeltaPositions, SwerveDriveKinematics kinematics){
        Twist2d twist = kinematics.toTwist2d(swerveModuleDeltaPositions);
        simRotation = simRotation.plus(new Rotation2d(twist.dtheta));
        Logger.recordOutput("TEST", kinematics.toString());
    }

    @Override
    public void resetHeading(){
        
    }
}
