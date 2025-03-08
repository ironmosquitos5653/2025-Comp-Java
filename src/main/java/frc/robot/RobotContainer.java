// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Position;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeSpitCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbOutCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralSpitCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LineUpCommand;
import frc.robot.commands.LineUpCommandStation;
import frc.robot.commands.TestPID;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.AutonomousManager;
import frc.robot.subsystems.vision.TrajectoryCommandFactory;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final VisionSubsystem visionSubsystem;
  private final ClimbSubsystem climbSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private TrajectoryCommandFactory trajectoryCommandFactory;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController coPilotController = new CommandXboxController(1);

  private final AutonomousManager autonomousManager;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        visionSubsystem = new VisionSubsystem(drive);
        trajectoryCommandFactory = new TrajectoryCommandFactory(drive, visionSubsystem);
        climbSubsystem = new ClimbSubsystem();
        elevatorSubsystem = new ElevatorSubsystem(driveController);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        visionSubsystem = null;
        climbSubsystem = null;
        elevatorSubsystem = null;
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        visionSubsystem = null;
        climbSubsystem = null;
        elevatorSubsystem = null;
        break;
    }
    autonomousManager = new AutonomousManager(drive, elevatorSubsystem, trajectoryCommandFactory);
    autonomousManager.initialize();
    // trajectoryCommandFactory = new TrajectoryCommandFactory(drive, visionSubsystem);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /*
     * Elevator buttons:
     * Algae shoot high, algae shoot low, level 1, level 2, level 3, and level 4 positions.
     *
     * Coral buttons:
     * Intake and shoot.
     *
     * Climb
     * Deploy, intake,
     */

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));
    visionSubsystem.setDefaultCommand(new RunCommand(() -> {}, visionSubsystem));
    // climbSubsystem.setDefaultCommand(new RunCommand(() -> {}, climbSubsystem));
    elevatorSubsystem.setDefaultCommand(new RunCommand(() -> {}, elevatorSubsystem));

    driveController.y().onTrue(new AlgaeIntakeCommand(elevatorSubsystem)); // Reset
    driveController.b().onTrue(new AlgaeSpitCommand(elevatorSubsystem)); // spit
    driveController.a().onTrue(new LineUpCommandStation(trajectoryCommandFactory, visionSubsystem));
    driveController.povLeft().onTrue(Commands.runOnce((() -> elevatorSubsystem.reset())));
    driveController.povRight().whileTrue(new TestPID(elevatorSubsystem, climbSubsystem));
    driveController.povDown().onTrue(new ClimbOutCommand(climbSubsystem));
    driveController.povUp().whileTrue(new ClimbCommand(climbSubsystem));

    // Controls
    coPilotController
        .x()
        .onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(Position.ELV_1)));
    coPilotController
        .a()
        .onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(Position.ELV_2)));
    coPilotController
        .b()
        .onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(Position.ELV_3)));
    coPilotController
        .y()
        .onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(Position.ELV_4)));
    coPilotController
        .povUp()
        .onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(Position.ELV_4_algae)));
    coPilotController
        .povDown()
        .onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(Position.ELV_4_algaeLow)));

    coPilotController.rightBumper().onTrue(new CoralSpitCommand(elevatorSubsystem));

    // LineUp Commands
    driveController
        .leftBumper()
        .onTrue(new LineUpCommand(trajectoryCommandFactory, visionSubsystem, true));
    driveController
        .rightBumper()
        .onTrue(new LineUpCommand(trajectoryCommandFactory, visionSubsystem, false));

    coPilotController.leftBumper().onTrue(new CoralIntakeCommand(elevatorSubsystem));
    coPilotController.rightBumper().onTrue(new CoralSpitCommand(elevatorSubsystem));
    // coPilotController.povRight().onTrue(new AlgaeLowCommand(elevatorSubsystem));
    // coPilotController.povDown().onTrue(Commands.runOnce(() -> climbSubsystem.toggle()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
