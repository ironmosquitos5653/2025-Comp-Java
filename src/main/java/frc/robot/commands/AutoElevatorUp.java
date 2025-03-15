// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Position;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.vision.ReefSide;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoElevatorUp extends Command {
  ElevatorSubsystem m_ElevatorSubsystem;
  double waitTime;
  ReefSide m_ReefSide;

  public AutoElevatorUp(ElevatorSubsystem elevatorSubsystem, String reefSide, boolean left) {
    m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ElevatorSubsystem);
    this.waitTime = waitTime;
    m_ReefSide = null;
    if (reefSide.startsWith("Blue")) {
      for (ReefSide rs : VisionSubsystem.blueReefSidess)
    }
  }

  Timer timer;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(waitTime)) {
      m_ElevatorSubsystem.setPosition(Position.ELV_4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(waitTime + .2);
  }
}
