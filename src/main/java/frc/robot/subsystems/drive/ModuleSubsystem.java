// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.drive.ModuleIOBase;

public class ModuleSubsystem extends SubsystemBase {

  ModuleIOBase io;

  /** Creates a new ExampleSubsystem. */
  public ModuleSubsystem(ModuleIOBase io) {
    this.io = io;
  }

  public void setDesiredState(SwerveModuleState state, boolean isOpenLoop){
    io.setDesiredState(state, isOpenLoop);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
