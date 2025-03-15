// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Position;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeCommand extends Command {

  ElevatorSubsystem m_elevatorSubsystem;
  Position m_Position;
  Position m_starPosition;
  public static boolean interrupt = false;

  Timer timer;

  public AlgaeIntakeCommand(ElevatorSubsystem elevatorSubsystem, Position position) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_Position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Position.Algae_tmp.position = m_Position.position;
    m_starPosition = Position.Algae_tmp;

    m_elevatorSubsystem.setPosition(m_starPosition);

    timer = new Timer();
    timer.start();
    interrupt = false;
    m_elevatorSubsystem.setCurrentLimit(12);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.setAlgaeSpeed(-1);
    double diff = m_elevatorSubsystem.getElevatorPosition() - m_starPosition.position;
    if (diff < 2 && diff > -2) {
      m_elevatorSubsystem.setPosition(m_Position);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setPosition(m_starPosition);
    if (interrupt) {
      m_elevatorSubsystem.reset();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("AlgaeSpeed", m_elevatorSubsystem.getAlgaeVelocity());
    SmartDashboard.putString(
        "algaeFinished",
        interrupt
            + " - "
            + (m_elevatorSubsystem.getAlgaeVelocity() < .5)
            + " - "
            + m_elevatorSubsystem.getAlgaeVelocity()
            + " - "
            + timer.hasElapsed(1)
            + (interrupt || (m_elevatorSubsystem.getAlgaeVelocity() < .5 && timer.hasElapsed(1))));
    return interrupt || (m_elevatorSubsystem.getAlgaeVelocity() > -.5 && timer.hasElapsed(1));
  }
}
