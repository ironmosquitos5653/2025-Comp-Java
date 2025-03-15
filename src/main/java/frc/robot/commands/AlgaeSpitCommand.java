// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Position;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeSpitCommand extends Command {
  ElevatorSubsystem m_elevatorSubsystem;

  public AlgaeSpitCommand(ElevatorSubsystem elevatorSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
  }

  Timer timer;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.setPosition(Position.Algae_Spit);
    timer = null;
    m_elevatorSubsystem.setCurrentLimit(80);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_elevatorSubsystem.getElevatorPosition() > Position.Algae_Spit.position - 1) {
      m_elevatorSubsystem.setAlgaeSpeed(1);
      if (timer == null) {
        timer = new Timer();
        timer.start();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setPosition(Position.ELV_off);
    m_elevatorSubsystem.setAlgaeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer != null && timer.hasElapsed(1);
  }
}
