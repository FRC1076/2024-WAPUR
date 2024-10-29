package frc.robot.subsystems.drive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private GyroIO gyroIO;
    private Module[] modules = new Module[4];

    public DriveSubsystem(
        GyroIO gyroIO,
        Module FrontLeftModule,
        Module FrontRightModule,
        Module RearLeftModule,
        Module RearRightModule
    ) {
        this.gyroIO = gyroIO;
        this.modules[0] = FrontLeftModule;
        this.modules[1] = FrontRightModule;
        this.modules[2] = RearLeftModule;
        this.modules[3] = RearRightModule;
    }
}
