// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.Constants.DriveConstants.ModuleConstants.Corner;
import frc.robot.utils.units.UnitConversion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Velocity;
import frc.robot.utils.units.UnitConversion;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class Module {

    ModuleIO io;
    Corner ModuleID;
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    /** Creates a new Module. */
    public Module(ModuleIO io,Corner ModuleID) {
        this.io = io;
        this.ModuleID = ModuleID;
        updateTurnEncoder();
    }

    public void setDesiredState(SwerveModuleState state, boolean isOpenLoop){

        io.updateInputs(inputs); // Gets latest values from the IO layer

        Rotation2d encoderRotation = inputs.turnPosition;

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState optState = SwerveModuleState.optimize(state, encoderRotation);

        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        optState.speedMetersPerSecond *= optState.angle.minus(encoderRotation).getCos();

        if(!isOpenLoop){
            //In closed loop drive, use on board pid controller for drive motor
            io.setDriveVelocity(
                MetersPerSecond.of(optState.speedMetersPerSecond).in(MetersPerSecond)
            );
        }
        else{
            // Divide the drive output by the max speed to scale it from -1 to 1 and make it open loop
            io.setDriveVoltage(
                (optState.speedMetersPerSecond / ModuleConstants.Common.kMaxModuleSpeed.in(MetersPerSecond)) * ModuleConstants.Common.kMaxDriveVolts.in(Volts)
            );
        }

        io.setTurnPosition(
            optState.angle
        );
    }

    /**Returns the angle of this swerve module*/
    public Rotation2d getAngle(){
        return inputs.turnPosition;
    }

    /**Returns a double with the drive motor's characterization position, in radians */
    public double getCharacterizationPositionRads() {
        return inputs.drivePositionMeters;
    }

    /**Returns a Measure object with the distance that this swerve module has driven*/
    public Measure<Distance> getPosition() {
        return Meters.of(inputs.drivePositionMeters);
    }

    /**Returns a double with the distance that this swerve module has driven, in meters*/
    public double getPositionMeters() {
        return getPosition().in(Meters);
    }

    /**Returns a Measure object with the drive motor's current linear velocity */
    public Measure<Velocity<Distance>> getVelocity() {
        return MetersPerSecond.of(inputs.driveVelocityMetersPerSec);
    }

    /**Returns a double with the drive motor's current linear velocity, in meters per sec*/
    public double getVelocityMetersPerSec() {
        return getVelocity().in(MetersPerSecond);
    }


    /**Returns a double with the angular velocity of this swerve module's wheel, in radians per second*/
    public double getCharacterizationVelocityRadsPerSec() {
        return inputs.driveVelocityMetersPerSec;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(),getAngle());
    }

    /**Returns the swerve module position (drive motor position and turn angle) */
    public SwerveModulePosition getSwervePosition() {
        return new SwerveModulePosition(getPosition(),getAngle());
    }

    /**Updates relative turn encoder with value from absolute encoder */
    public void updateTurnEncoder() {
        io.updateTurnEncoder();
    }
    
    // Module is not a subsystem so this won't be default called
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive/" + ModuleID.label,inputs);
        io.periodic();
    }

}
