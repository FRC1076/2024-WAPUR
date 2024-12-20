// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants.ModuleConstants.Corner;
import static frc.robot.Constants.ElevatorConstants.autonHeight;
import static frc.robot.Constants.ElevatorConstants.lowHeight;
import static frc.robot.Constants.ElevatorConstants.midHeight;
import static frc.robot.Constants.ElevatorConstants.highHeight;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OIConstants.Driver;
import frc.robot.Constants.OIConstants.Operator;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drive.DriveClosedLoopTeleop;
import frc.robot.commands.elevator.SetElevatorVelocity;
import frc.robot.commands.grabber.GrabberEject;
import frc.robot.commands.grabber.GrabberIntake;
import frc.robot.commands.grabber.GrabberStop;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIOHardware;
import frc.robot.subsystems.drive.ModuleIOHardware;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberIOHardware;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterSubsystem;


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

    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem(new ElevatorIOHardware());

    private final GrabberSubsystem m_grabber = new GrabberSubsystem(new GrabberIOHardware());

    private final IntakeSubsystem m_intake = new IntakeSubsystem(new IntakeIOHardware()); 

    private final ShooterSubsystem m_shooter = new ShooterSubsystem(new ShooterIOHardware());

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OIConstants.Driver.kControllerPort);
    
    private final Trigger dsEnabledTrigger = 
        new Trigger(() -> DriverStation.isEnabled());

    private final CommandXboxController m_operatorController = 
        new CommandXboxController(OIConstants.Operator.kControllerPort);

    private final SendableChooser<Command> m_autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        NamedCommands.registerCommand("startShoot", 
            new ParallelCommandGroup(
                new RunShooter(m_shooter),
                new SequentialCommandGroup(
                    new WaitCommand(1.0),
                    new RunCommand(() -> m_shooter.setServoAngleDeg(90))
                )
            ));
        NamedCommands.registerCommand("stopShoot", 
            new ParallelCommandGroup(
                new StopShooter(m_shooter),
                new RunCommand(() -> m_shooter.setServoAngleDeg(180))
            ));
        NamedCommands.registerCommand("raiseElevator", new RunCommand(() -> m_elevator.setPosition(autonHeight)));

        // Configure the trigger bindings
        configureBindings();

        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(m_autoChooser);
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
        //Example
        new Trigger(m_exampleSubsystem::exampleCondition)
            .onTrue(new ExampleCommand(m_exampleSubsystem));
        
        //Intake
        m_operatorController.leftTrigger(Operator.kControllerTriggerThreshold).whileTrue(new RunIntake(m_intake));
        
        //Shooter

        //There is a delay to stopping the shooter because the servo may have a delay; I (Jesse Kane) am good at spelling.
        m_operatorController.rightTrigger(Operator.kControllerTriggerThreshold).whileTrue(
            new ParallelCommandGroup(
                new RunShooter(m_shooter),
                new SequentialCommandGroup(
                    new WaitCommand(1.0),
                    new RunCommand(() -> m_shooter.setServoAngleDeg(90))
                )
            )
        ).whileFalse(
            new ParallelCommandGroup(
                new StopShooter(m_shooter),
                new RunCommand(() -> m_shooter.setServoAngleDeg(180))
            )
        );
        /*m_operatorController.rightTrigger(Operator.kControllerTriggerThreshold).whileTrue(new RunCommand(() -> m_shooter.setServoAngleDeg(90), m_shooter))
                            .whileFalse(new RunCommand(() -> m_shooter.setServoAngleDeg(180)));*/

        //Elevator Joystick Control
        m_elevator.setDefaultCommand(
            /*new SetElevatorVelocity(
                m_elevator, 
                () -> MathUtil.applyDeadband(-m_operatorController.getLeftY(), Operator.kControllerDeadband) * Operator.kElevatorManualSpeedLimit
            )*/
            new RunCommand(() -> m_elevator.setVoltage(MathUtil.applyDeadband(-m_operatorController.getLeftY() * 6, Driver.kControllerDeadband)), m_elevator)
        );

        //Elevator Presets
        m_operatorController.a().whileTrue(new RunCommand(() -> m_elevator.setPosition(lowHeight), m_elevator));
        m_operatorController.b().whileTrue(new RunCommand(() -> m_elevator.setPosition(midHeight), m_elevator));
        m_operatorController.y().whileTrue(new RunCommand(() -> m_elevator.setPosition(highHeight), m_elevator));

        //Grabber Eject
        m_operatorController.rightBumper()
            .whileTrue(
                new GrabberEject(m_grabber)
            ).
            onFalse(
                new GrabberStop(m_grabber)
            );

        //Grabber Intake
        m_operatorController.leftBumper()
            .whileTrue(
                new GrabberIntake(m_grabber)
            ).
            onFalse(
                new GrabberStop(m_grabber)
            );
        
        //Driver Clutch
        m_DriveSubsystem.setDefaultCommand(
            new DriveClosedLoopTeleop(
                () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), Driver.kControllerDeadband),
                () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), Driver.kControllerDeadband), 
                () -> MathUtil.applyDeadband(-m_driverController.getRightX(), Driver.kControllerDeadband), 
                m_DriveSubsystem)
        );

        m_driverController.leftBumper().whileTrue(
            new DriveClosedLoopTeleop(
                () -> {return MathUtil.applyDeadband(-m_driverController.getLeftY(), Driver.kControllerDeadband) * DriveConstants.kDoubleClutchTranslation;},
                () -> {return MathUtil.applyDeadband(-m_driverController.getLeftX(), Driver.kControllerDeadband) * DriveConstants.kDoubleClutchTranslation;}, 
                () -> {return MathUtil.applyDeadband(-m_driverController.getRightX(), Driver.kControllerDeadband) * DriveConstants.kDoubleClutchRotation;}, 
                m_DriveSubsystem)
        );

        m_driverController.rightBumper().whileTrue(
            new DriveClosedLoopTeleop(
                () -> {return MathUtil.applyDeadband(-m_driverController.getLeftY(), Driver.kControllerDeadband) * DriveConstants.kSingleClutchTranslation;},
                () -> {return MathUtil.applyDeadband(-m_driverController.getLeftX(), Driver.kControllerDeadband) * DriveConstants.kSingleClutchTranslation;}, 
                () -> {return MathUtil.applyDeadband(-m_driverController.getRightX(), Driver.kControllerDeadband) * DriveConstants.kSingleClutchRotation;}, 
                m_DriveSubsystem)
        );

        m_driverController.rightTrigger().whileTrue(
            new DriveClosedLoopTeleop(
                () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), Driver.kControllerDeadband),
                () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), Driver.kControllerDeadband), 
                () -> {return MathUtil.applyDeadband(-m_driverController.getRightX(), Driver.kControllerDeadband) * DriveConstants.kSingleClutchRotation;}, 
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


    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
