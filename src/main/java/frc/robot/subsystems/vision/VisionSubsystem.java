// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
  private Drive m_driveSubsystem;
  /** Creates a new VisionSubsystem. */
  private final Field2d field2d = new Field2d();

  private Transform3d cameraPose =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-12.5), Units.inchesToMeters(10), Units.inchesToMeters(0)),
          new Rotation3d(0, 0, 0));

  public VisionSubsystem(Drive driveSubsystem) {

    initReefss();
    m_driveSubsystem = driveSubsystem;
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(4, 0);

    tab.addString("Pose2", this::getFomattedPose2).withPosition(0, 3).withSize(2, 0);
    tab.add("Field", field2d).withPosition(3, 0).withSize(6, 4);
  }

  @Override
  public void periodic() {
    findClosest();
    boolean useMegaTag2 = !true; // set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if (useMegaTag2 == false) {
      LimelightHelpers.PoseEstimate mt1 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-high");
      if (mt1 != null) {
        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
          if (mt1.rawFiducials[0].ambiguity > .7) {
            doRejectUpdate = true;
          }
          if (mt1.rawFiducials[0].distToCamera > 3) {
            doRejectUpdate = true;
          }
        }
        if (mt1.tagCount == 0) {
          doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
          SmartDashboard.putString("mt1", getFomattedPose(mt1.pose));
          m_driveSubsystem.addVisionMeasurement(
              cameraTransform(mt1.pose),
              mt1.timestampSeconds,
              VecBuilder.fill(.1, .1, .1)); // 9999999));
          SmartDashboard.putNumber("mt1X", mt1.timestampSeconds);
        }
      }
    } else if (useMegaTag2 == true) {
      LimelightHelpers.SetRobotOrientation(
          "limelight-high", m_driveSubsystem.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-high");

      if (mt2 != null) {

        if (Math.abs(m_driveSubsystem.getTurnRate())
            > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision
        // updates
        {
          doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
          doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
          m_driveSubsystem.addVisionMeasurement(
              cameraTransform(mt2.pose), mt2.timestampSeconds, VecBuilder.fill(.5, .5, .5));
        }
      }
    }
    updateField();
  }

  public Pose2d cameraTransform(Pose2d pose) {
    return new Pose3d(pose).transformBy(cameraPose).toPose2d();
  }

  public void updateField() {
    field2d.setRobotPose(m_driveSubsystem.getPose());
  }

  private String getFomattedPose() {
    return getFomattedPose(m_driveSubsystem.getPose());
  }

  private String getFomattedPose2() {
    return getFomattedPose(m_driveSubsystem.getPose());
  }

  private String getFomattedPose(Pose2d pose) {

    return String.format(
        "(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  private List<ReefSide> blueReefSidess;
  private List<Pose2d> coralStationPoints;

  private void initReefss() {
    blueReefSidess = new ArrayList<ReefSide>();
    coralStationPoints = new ArrayList<Pose2d>();
    blueReefSidess.add(
        new ReefSide(
            new Pose2d(new Translation2d(6.971, 4), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(5.97, 3.8),
                new Rotation2d(Units.degreesToRadians(180))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(6.04, 4.22),
                new Rotation2d(Units.degreesToRadians(180))), // Right Position (On Reef)
            "BlueRight")); // AT 21
    blueReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(5.850, 1.919), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(5.58, 1.91),
                new Rotation2d(Units.degreesToRadians(120))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(5.89, 2.11),
                new Rotation2d(Units.degreesToRadians(120))), // Right Position (On Reef)
            "BlueRightBottom")); // AT 22
    blueReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(2.915, 1.558), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(3.3, 1.92),
                new Rotation2d(Units.degreesToRadians(120))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(3.68, 1.72),
                new Rotation2d(Units.degreesToRadians(120))), // Right Position (On Reef)
            "BlueLeftBottom")); // AT 17
    blueReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(2.174, 4.064), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(2.05, 3.98),
                new Rotation2d(Units.degreesToRadians(180))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(2.13, 3.62),
                new Rotation2d(Units.degreesToRadians(180))), // Right Position (On Reef)
            "BlueLeft")); // AT 18

    blueReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(3.042, 6.355), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(3.25, 5.98),
                new Rotation2d(Units.degreesToRadians(180))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(3.53, 6.24),
                new Rotation2d(Units.degreesToRadians(180))), // Right Position (On Reef)
            "BlueRightTop")); // AT 19

    blueReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(5.957, 6.316), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(5.65, 6.17),
                new Rotation2d(Units.degreesToRadians(180))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(5.22, 6.37),
                new Rotation2d(Units.degreesToRadians(180))), // Right Position (On Reef)
            "BlueLeftTop")); // AT 20

    coralStationPoints.add(
        new Pose2d(
            new Translation2d(1.648, .807),
            new Rotation2d(Units.degreesToRadians(-120)))); // Blue Bottom
    coralStationPoints.add(
        new Pose2d(
            new Translation2d(1.697, 7.184),
            new Rotation2d(Units.degreesToRadians(120)))); // Blue Top
    coralStationPoints.add(
        new Pose2d(
            new Translation2d(16.14, 1.04),
            new Rotation2d(Units.degreesToRadians(-60)))); // Red Bottom
    coralStationPoints.add(
        new Pose2d(
            new Translation2d(16.087, 6.999),
            new Rotation2d(Units.degreesToRadians(60)))); // Red Top
  }

  public ReefSide findClosest() {
    Pose2d pose = m_driveSubsystem.getPose();
    ReefSide closest = null;
    double distance = 100;
    if (blueReefSidess != null) {
      for (ReefSide p : blueReefSidess) {
        double d = p.getSidePosition().getTranslation().getDistance(pose.getTranslation());
        if (d < distance) {
          distance = d;
          closest = p;
        }
      }
      SmartDashboard.putString("Closest", closest.getDescription());
    }
    return closest;
  }

  public Pose2d findClosestCoralStation() {
    Pose2d pose = m_driveSubsystem.getPose();
    Pose2d closest = null;
    double distance = 100;
    if (coralStationPoints != null) {
      for (Pose2d p : coralStationPoints) {
        double d = p.getTranslation().getDistance(pose.getTranslation());
        if (d < distance) {
          distance = d;
          closest = p;
        }
      }
    }
    return closest;
  }
}
