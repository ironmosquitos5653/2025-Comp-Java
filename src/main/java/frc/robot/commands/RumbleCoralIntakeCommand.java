// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Position;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RumbleCoralIntakeCommand extends Command {

  ElevatorSubsystem m_elevatorSubsystem;

  public RumbleCoralIntakeCommand(ElevatorSubsystem elevatorSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
  }

  Timer timer = null;

  public static boolean interrupt = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    interrupt = false;
    m_elevatorSubsystem.setPosition(Position.ELV_Intake);
    timer = new Timer();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.coralIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted && !interrupt) {
      m_elevatorSubsystem.coralTravel();
      m_elevatorSubsystem.setPosition(Position.ELV_IntakeTravel);
      m_elevatorSubsystem.setRumble();
    } else {
      m_elevatorSubsystem.reset();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return interrupt || (timer.hasElapsed(1) && m_elevatorSubsystem.getCoralVelocity() < 100);
  }
}
