// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Position;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.ReefSide;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.List;

public class AutoElevatorUp extends Command {
  ElevatorSubsystem m_ElevatorSubsystem;
  Drive m_Drive;
  double waitTime;
  Pose2d m_targetPose;
  double m_distance;

  public AutoElevatorUp(
      ElevatorSubsystem elevatorSubsystem,
      Drive drive,
      String reefSide,
      boolean left,
      double distance) {
    m_ElevatorSubsystem = elevatorSubsystem;
    m_Drive = drive;
    m_distance = distance;
    addRequirements(m_ElevatorSubsystem);
    this.waitTime = 1.1;

    if (reefSide.startsWith("Blue")) {
      m_targetPose = getTargetPose(VisionSubsystem.blueReefSidess, reefSide, left);
    } else {
      m_targetPose = getTargetPose(VisionSubsystem.redReefSidess, reefSide, left);
    }
  }

  private Pose2d getTargetPose(List<ReefSide> reefs, String reefSide, boolean left) {
    for (ReefSide rs : reefs) {
      if (rs.getDescription() == reefSide) {
        if (left) {
          return rs.getLeftPosition();
        } else {
          return rs.getLeftPosition();
        }
      }
    }
    return null;
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
      // m_ElevatorSubsystem.setPosition(Position.ELV_4);
    }
    SmartDashboard.putNumber(
        "GetDistance",
        m_Drive.getPose().getTranslation().getDistance(m_targetPose.getTranslation()));
    SmartDashboard.putString("Target Pose", m_targetPose.getX() + " - " + m_targetPose.getY());
    Pose2d current = m_Drive.getPose();
    SmartDashboard.putString("Current Pose", current.getX() + " - " + current.getY());

    if (m_Drive.getPose().getTranslation().getDistance(m_targetPose.getTranslation())
        < m_distance) {
      m_ElevatorSubsystem.setPosition(Position.ELV_4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Position.ELV_4.position < m_ElevatorSubsystem.getElevatorPosition() + 1;
  }
}
