package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.Physical.*;
import static edu.wpi.first.units.Units.*;
import frc.robot.subsystems.drive.Module;
import frc.robot.Constants.DriveConstants.ModuleConstants.Corner;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveSubsystem extends SubsystemBase {


    private GyroIO gyroIO;
    private Module[] modules = new Module[4];
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d gyroRotation = new Rotation2d();
    private SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(
        kinematics, 
        gyroRotation, 
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        }, 
        new Pose2d()
    );

    public DriveSubsystem(
        GyroIO gyroIO,
        ModuleIO FrontLeftModuleIO,
        ModuleIO FrontRightModuleIO,
        ModuleIO RearLeftModuleIO,
        ModuleIO RearRightModuleIO
    ) {
        this.gyroIO = gyroIO;
        this.modules[0] = new Module(FrontLeftModuleIO, Corner.FrontLeft);
        this.modules[1] = new Module(FrontRightModuleIO, Corner.FrontRight);
        this.modules[2] = new Module(RearLeftModuleIO, Corner.RearLeft);
        this.modules[3] = new Module(RearRightModuleIO, Corner.RearRight);
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = this.modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = this.modules[i].getSwervePosition();
        }
        return positions;
    }

    /** Drives based on the given CHASSIS-ORIENTED ChassisSpeeds object */
    public void drive(ChassisSpeeds speeds, boolean isOpenLoop){
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.ModuleConstants.Common.kMaxModuleSpeed);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(setpointStates[i],isOpenLoop);
        }

        // Log setpoint states, and voltages if Open Loop
        if (isOpenLoop){
            double[] voltages = {
                speedToVoltage(setpointStates[0].speedMetersPerSecond),
                speedToVoltage(setpointStates[1].speedMetersPerSecond),
                speedToVoltage(setpointStates[2].speedMetersPerSecond),
                speedToVoltage(setpointStates[3].speedMetersPerSecond),
            };
            Logger.recordOutput("SwerveStates/VoltagesOL", voltages);
        }
        Logger.recordOutput("SwerveStates/Setpoints",setpointStates);
    }

    /** Drives based on the given FIELD-ORIENTED ChassisSpeeds object */
    public void driveFO(ChassisSpeeds speeds, boolean isOpenLoop) {
        ChassisSpeeds COSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyroRotation);
        drive(COSpeeds,isOpenLoop);
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Stops the drivetrain */
    public void stop() {
        drive(new ChassisSpeeds(),false);
    }

    @Override
    public void periodic() {
        //Logs to advantageKit and runs periodic routines on modules
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }

        // Updates GyroRotation
        gyroRotation = gyroInputs.yawPosition;

        // Updates odometry
        estimator.update(gyroRotation,getModulePositions());

    }

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(kWheelBase.divide(2.0), kTrackWidth.divide(2.0)),
            new Translation2d(kWheelBase.divide(2.0), kTrackWidth.divide(-2.0)),
            new Translation2d(kWheelBase.divide(-2.0), kTrackWidth.divide(2.0)),
            new Translation2d(kWheelBase.divide(-2.0), kTrackWidth.divide(-2.0))
        };
    }

    /** Converts a closed-loop speed from a ModuleState object to an open-loop voltage */
    private static double speedToVoltage(double speedMetersPerSecond) {
        return (speedMetersPerSecond/DriveConstants.ModuleConstants.Common.kMaxModuleSpeed.in(MetersPerSecond));
    }


}
