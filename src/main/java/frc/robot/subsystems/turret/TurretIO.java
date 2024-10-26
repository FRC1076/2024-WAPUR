package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double turretPositionRad = 0.0;
        public double turretPositionRadPerSec = 0.0;

        public double turretAppliedVolts = 0.0;
        public double[] turretCurrentAmps = new double[] {};
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    /* Turn turret in open-loop */
    public default void setVoltage(double turretVolts) {}

    /* Turn closed-loop to specified position */
    public default void setPositionRad(double turretPositionRads, double turretFFVolts) {}

}