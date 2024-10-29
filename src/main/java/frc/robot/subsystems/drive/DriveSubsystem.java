package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Module;
import frc.robot.Constants.DriveConstants.ModuleConstants.Corner;
import com.pathplanner.lib.auto.AutoBuilder;

public class DriveSubsystem extends SubsystemBase {
    private GyroIO gyroIO;
    private Module[] modules = new Module[4];
    private GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

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

    public getState


}
