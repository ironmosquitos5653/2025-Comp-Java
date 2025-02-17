// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Position;
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
    // wait 1s
    register("SpitCoral", new CoralSpitCommand(m_elevatorSubsystem));
    register("IntakeOn", new CoralIntakeCommand(m_elevatorSubsystem));
  }

  private void register(String name, Command command) {
    NamedCommands.registerCommand(name, command);
  }
}
