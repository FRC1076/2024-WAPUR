package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.FeetPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.DriveConstants.kMaxRotSpeed;
import frc.robot.subsystems.drive.DriveSubsystem;
/**
 * Drives Teleop in closed loop mode from controller inputs
 */
public class DriveClosedLoopTeleop extends Command {
    private final DriveSubsystem m_subsystem;
    private final DoubleSupplier xTransSpeedSupplier; // field-oriented x translational speed, scaled from -1.0 to 1.0
    private final DoubleSupplier yTransSpeedSupplier; // field-oriented y translational speed, scaled from -1.0 to 1.0
    private final DoubleSupplier omegaSupplier; // rotational speed, scaled from -1.0 to 1.0
    
    public DriveClosedLoopTeleop(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, DriveSubsystem subsystem) {
        this.xTransSpeedSupplier = xSupplier;
        this.yTransSpeedSupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(
            FeetPerSecond.of(xTransSpeedSupplier.getAsDouble() * 3),
            FeetPerSecond.of(yTransSpeedSupplier.getAsDouble() * 3),
            kMaxRotSpeed.times(omegaSupplier.getAsDouble())
        );
        m_subsystem.drive(
            speeds,
            false
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stop();
    }


}