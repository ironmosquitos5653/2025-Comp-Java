// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Position;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeSpitCommand;
import frc.robot.commands.AutoElevatorUp;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralSpitCommand;
import frc.robot.commands.SetCoralCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class AutonomousManager {
  Drive m_Drive;
  ElevatorSubsystem m_elevatorSubsystem;
  TrajectoryCommandFactory m_trajectoryCommandFactory;

  public AutonomousManager(
      Drive drive,
      ElevatorSubsystem elevatorSubsystem,
      TrajectoryCommandFactory trajectoryCommandFactory) {
    m_Drive = drive;
    m_elevatorSubsystem = elevatorSubsystem;
    m_trajectoryCommandFactory = trajectoryCommandFactory;
  }

  public void initialize() {
    register("SetCoral", new SetCoralCommand(m_elevatorSubsystem));
    register(
        "LiftElevator", Commands.runOnce(() -> m_elevatorSubsystem.setPosition(Position.ELV_4)));
    register(
        "LiftElevatorAuto",
        Commands.runOnce(() -> m_elevatorSubsystem.setPosition(Position.ELV_4_auto)));
    // wait 1s
    register("SpitCoral", new CoralSpitCommand(m_elevatorSubsystem));
    register("IntakeOn", new CoralIntakeCommand(m_elevatorSubsystem));
    register("AlgaeIntakeOn", new AlgaeIntakeCommand(m_elevatorSubsystem, Position.ELV_4_algae));
    register("AlgaeSpit", new AlgaeSpitCommand(m_elevatorSubsystem));
    register("Reset", Commands.runOnce(() -> m_elevatorSubsystem.reset()));
    register(
        "LiftElevator3", Commands.runOnce(() -> m_elevatorSubsystem.setPosition(Position.ELV_3)));

    // RED RIGHT
    register(
        "AutoElevatorUpRLTL",
        new AutoElevatorUp(m_elevatorSubsystem, m_Drive, "RedLeftTop", true, 3.35));
    register(
        "AutoElevatorUpRRTL",
        new AutoElevatorUp(m_elevatorSubsystem, m_Drive, "RedRightTop", true, 3));
    register(
        "AutoElevatorUpRRTR",
        new AutoElevatorUp(m_elevatorSubsystem, m_Drive, "RedRightTop", false, 3));

    // RED LEFT
    register(
        "AutoElevatorUpRLBR",
        new AutoElevatorUp(m_elevatorSubsystem, m_Drive, "RedLeftBottom", false, 3.3));
    register(
        "AutoElevatorUpRLBL",
        new AutoElevatorUp(m_elevatorSubsystem, m_Drive, "RedRightBottom", true, 2.8));
    register(
        "AutoElevatorUpRLBR",
        new AutoElevatorUp(m_elevatorSubsystem, m_Drive, "RedRightBottom", false, 2.7));

    // BLUE RIGHT
    register(
        "AutoElevatorUpBRBL",
        new AutoElevatorUp(m_elevatorSubsystem, m_Drive, "BlueRightBottom", true, 3.35));
    register(
        "AutoElevatorUpBLBL",
        new AutoElevatorUp(m_elevatorSubsystem, m_Drive, "BlueLeftBottom", true, 3));
    register(
        "AutoElevatorUpBLBR",
        new AutoElevatorUp(m_elevatorSubsystem, m_Drive, "BlueLeftBottom", false, 3));

    // BLUE LEFT
    register(
        "AutoElevatorUpBRTR",
        new AutoElevatorUp(m_elevatorSubsystem, m_Drive, "BlueRightTop", false, 3.35));
    register(
        "AutoElevatorUpBLTL",
        new AutoElevatorUp(m_elevatorSubsystem, m_Drive, "BlueLeftTop", true, 3));
    register(
        "AutoElevatorUpBLTR",
        new AutoElevatorUp(m_elevatorSubsystem, m_Drive, "BlueLeftTop", false, 3));
  }

  private void register(String name, Command command) {
    NamedCommands.registerCommand(name, command);
  }
}
