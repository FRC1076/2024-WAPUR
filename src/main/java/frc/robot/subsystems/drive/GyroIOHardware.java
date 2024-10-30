package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import frc.robot.Constants.DriveConstants.GyroConstants;

public class GyroIOHardware implements GyroIO {

    private final Pigeon2 pigeon = new Pigeon2(GyroConstants.kGyroPort);
    private final StatusSignal<Double> yaw = pigeon.getYaw();
    private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

    public GyroIOHardware(){
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(GyroConstants.Signal.kYawUpdateFrequencyHz);
        yawVelocity.setUpdateFrequency(GyroConstants.Signal.kYawVelocityUpdateFrequencyHz);
        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs){
        inputs.connected = BaseStatusSignal.refreshAll(yaw,yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = pigeon.getRotation2d();
        inputs.yawVelocity = RadiansPerSecond.of(pigeon.getAngularVelocityZWorld().getValueAsDouble());
    }
}
