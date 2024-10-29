package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import frc.robot.Constants.DriveConstants.GyroConstants;;

public class GyroIOHardware implements GyroIO {

    private final Pigeon2 pigeon = new Pigeon2(GyroConstants.kGyroPort);

    public GyroIOHardware(){}

    @Override
    public void updateInputs(GyroIOInputs inputs){
        inputs.yawPosition = pigeon.getRotation2d();
        inputs.yawVelocity = RadiansPerSecond.of(pigeon.getAngularVelocityZWorld().getValueAsDouble());
    }
}
