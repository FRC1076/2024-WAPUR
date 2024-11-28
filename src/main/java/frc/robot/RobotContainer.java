// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OIConstants.Driver;
import frc.robot.Constants.OIConstants.Operator;
import frc.robot.Constants.DriveConstants.ModuleConstants.Corner;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.elevator.SetElevatorPosition;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drive.DriveClosedLoopTeleop;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIOHardware;
import frc.robot.subsystems.drive.ModuleIOHardware;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeIOHardware;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.Constants.ElevatorConstants.floorHeight;
import static frc.robot.Constants.ElevatorConstants.rowTwoHeight;
import static frc.robot.Constants.ElevatorConstants.rowThreeHeight;

import edu.wpi.first.math.MathUtil;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem(
        new GyroIOHardware(),
        new ModuleIOHardware(Corner.FrontLeft),
        new ModuleIOHardware(Corner.FrontRight),
        new ModuleIOHardware(Corner.RearLeft),
        new ModuleIOHardware(Corner.RearRight)
    );

    private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem(new ElevatorIOHardware());

    private final IntakeSubsystem m_intake = new IntakeSubsystem(new IntakeIOHardware()); 

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OIConstants.Driver.kControllerPort);
    
    private final Trigger dsEnabledTrigger = 
        new Trigger(() -> DriverStation.isEnabled());

    private final CommandXboxController m_operatorController = 
        new CommandXboxController(OIConstants.Operator.kControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the

    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
    * predicate, or via the named factories in {@link
    * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
    * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
    * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
    * joysticks}.
    */
    private void configureBindings() {
        
        new Trigger(m_exampleSubsystem::exampleCondition)
            .onTrue(new ExampleCommand(m_exampleSubsystem));
        
        m_operatorController.leftTrigger(Operator.kControllerTriggerThreshold).whileTrue(new RunIntake(m_intake));

        m_DriveSubsystem.setDefaultCommand(
            new DriveClosedLoopTeleop(
                () -> MathUtil.applyDeadband(m_driverController.getLeftY(), Driver.kControllerDeadband),
                () -> MathUtil.applyDeadband(m_driverController.getLeftX(), Driver.kControllerDeadband), 
                () -> MathUtil.applyDeadband(m_driverController.getRightX(), Driver.kControllerDeadband), 
                m_DriveSubsystem)
        );

        // Reset Heading of swerve
        m_driverController.leftTrigger(Driver.kControllerTriggerThreshold).and(
            m_driverController.rightTrigger(Driver.kControllerTriggerThreshold).and(
                m_driverController.x()
            )
        ).onTrue(new InstantCommand(
            () -> m_DriveSubsystem.resetHeading()
        ));

        //Elevator controls
        m_operatorController.a().onTrue(new SetElevatorPosition(floorHeight, m_ElevatorSubsystem));
        m_operatorController.b().onTrue(new SetElevatorPosition(rowTwoHeight, m_ElevatorSubsystem));
        m_operatorController.y().onTrue(new SetElevatorPosition(rowThreeHeight, m_ElevatorSubsystem));
    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.exampleAuto(m_exampleSubsystem);
    }
}
