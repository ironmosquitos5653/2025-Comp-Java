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
public class CoralIntakeCommand extends Command {

  ElevatorSubsystem m_elevatorSubsystem;

  public CoralIntakeCommand(ElevatorSubsystem elevatorSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
  }

  Timer timer = null;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.setPosition(Position.ELV_Intake);
    timer = new Timer();
    timer.start();
    SmartDashboard.putBoolean("CoralOn", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.coralIn();
    SmartDashboard.putNumber("CoralCurrent", m_elevatorSubsystem.getCoralCurrent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.coralOff();
    SmartDashboard.putBoolean("CoralOn", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("coralVelocity", m_elevatorSubsystem.getCoralVelocity());
    return timer.hasElapsed(1) && m_elevatorSubsystem.getCoralVelocity() < 100;
  }
}
