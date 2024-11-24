package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    public void deriveGyro(SwerveModuleState[] swerveModuleStates, SwerveDriveKinematics kinematics){
        simRotation = simRotation.plus(Rotation2d.fromRadians(kinematics.toChassisSpeeds(swerveModuleStates).omegaRadiansPerSecond * 0.02));
        /*Twist2d twist = kinematics.toTwist2d(swerveModuleDeltaPositions);
        simRotation = simRotation.plus(new Rotation2d(twist.dtheta));
        */
        Logger.recordOutput("TEST/swerveModuleStates", swerveModuleStates);
        Logger.recordOutput("TEST/simRotation", simRotation);
        /*
        Logger.recordOutput("TEST/twist", twist);
        Logger.recordOutput("TEST/simRotation", simRotation);*/
    }

    @Override
    public void resetHeading(){
        
    }
}
