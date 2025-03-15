// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestPID extends Command {
  ElevatorSubsystem m_elevatorSubsystem;
  ClimbSubsystem m_climbsubsystem;

  public TestPID(ElevatorSubsystem elevatorSubsystem, ClimbSubsystem climbsubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_climbsubsystem = climbsubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.setRumble();
    // m_elevatorSubsystem.setSpeed(.2);
    m_climbsubsystem.setSpeed(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*m_elevatorSubsystem.setSpeed(0);
    m_climbsubsystem.setSpeed(0);*/
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
