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

  private final String reefCamera = "limelight-reef";
  private final String coralStationCamera = "limelight-coral";

  private final ArrayList<Integer> coralStationTags;

  private Transform3d reefCameraTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-9), Units.inchesToMeters(10.175), Units.inchesToMeters(0)),
          new Rotation3d(0, 0, 0));
  private Transform3d coralCameraTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-12.5), Units.inchesToMeters(10), Units.inchesToMeters(0)),
          new Rotation3d(0, 0, 0));

  public VisionSubsystem(Drive driveSubsystem) {

    m_driveSubsystem = driveSubsystem;
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(4, 0);

    tab.addString("Pose2", this::getFomattedPose2).withPosition(0, 3).withSize(2, 0);
    tab.add("Field", field2d).withPosition(3, 0).withSize(6, 4);
    coralStationTags = new ArrayList<>();

    coralStationTags.add(1);
    coralStationTags.add(2);
    coralStationTags.add(12);
    coralStationTags.add(13);
  }

  @Override
  public void periodic() {
    findClosest();
    updateCamera();
    // updateCamera("limelight-low", lowCameraTransform);
  }

  public void updateCamera() {

    boolean useMegaTag2 = !true; // set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if (useMegaTag2 == false) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(reefCamera);
      /* LimelightHelpers.PoseEstimate mt1CS =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(coralStationCamera);
      if (mt1CS != null
          && mt1CS.rawFiducials.length == 1
          && coralStationTags.contains(mt1CS.rawFiducials[0].id)
          && mt1CS.rawFiducials[0].distToCamera < 2
          && mt1CS.rawFiducials[0].ambiguity < .7) {
        SmartDashboard.putNumber("CoralAmbiguity", mt1CS.rawFiducials[0].ambiguity);
        SmartDashboard.putNumber("AprilTag", mt1CS.rawFiducials[0].id);
        m_driveSubsystem.addVisionMeasurement(
            cameraTransform(mt1CS.pose, coralCameraTransform),
            mt1CS.timestampSeconds,
            VecBuilder.fill(.1, .1, .1)); // 9999999));
      }
      */
      if (mt1 != null) {
        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
          if (mt1.rawFiducials[0].ambiguity > .7) {
            doRejectUpdate = true;
          }

          if (mt1.rawFiducials[0].distToCamera > 2) {
            doRejectUpdate = true;
          }
          if (mt1.rawFiducials[0].id == 12) {
            // doRejectUpdate = true;
          }
        }
        if (mt1.tagCount == 0) {
          doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
          SmartDashboard.putString("mt1", getFomattedPose(mt1.pose));
          SmartDashboard.putNumber("AprilTag", mt1.rawFiducials[0].id);
          SmartDashboard.putNumber("Amibuity", mt1.rawFiducials[0].ambiguity);
          m_driveSubsystem.addVisionMeasurement(
              cameraTransform(mt1.pose, reefCameraTransform),
              mt1.timestampSeconds,
              VecBuilder.fill(.1, .1, .1)); // 9999999));
        }
      }
    } else if (useMegaTag2 == true) {
      String camera = reefCamera;

      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(reefCamera);

      LimelightHelpers.SetRobotOrientation(
          camera, m_driveSubsystem.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

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
              cameraTransform(mt2.pose, reefCameraTransform),
              mt2.timestampSeconds,
              VecBuilder.fill(.5, .5, .5));
        }
      }
    }
    updateField();
  }

  public Pose2d cameraTransform(Pose2d pose, Transform3d cameraTransform) {
    return new Pose3d(pose).transformBy(cameraTransform).toPose2d();
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

  public static List<ReefSide> blueReefSidess = initBlueReefPoints();
  public static List<ReefSide> redReefSidess = initRedReefPoints();
  private List<Pose2d> coralStationPoints;

  private void initReefss() {
    initCoralStationPoints();
  }

  private static ArrayList<ReefSide> initBlueReefPoints() {
    ArrayList<ReefSide> blueReefSidess = new ArrayList<ReefSide>();

    blueReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(2.915, 1.558), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(3.61, 2.92),
                new Rotation2d(Units.degreesToRadians(60))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(3.88, 2.76),
                new Rotation2d(Units.degreesToRadians(60))), // Right Position (On Reef)
            "BlueLeftBottom")); // AT 17
    blueReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(2.174, 4.064), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(3.13, 4.27),
                new Rotation2d(Units.degreesToRadians(0))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(3.09, 3.92),
                new Rotation2d(Units.degreesToRadians(0))), // Right Position (On Reef)
            "BlueLeft")); // AT 18

    blueReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(3.042, 6.355), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(4.01, 5.34),
                new Rotation2d(Units.degreesToRadians(-60))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(3.78, 5.19),
                new Rotation2d(Units.degreesToRadians(-60))), // Right Position (On Reef)
            "BlueRightTop")); // AT 19

    blueReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(5.957, 6.316), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(5.37, 5.12),
                new Rotation2d(Units.degreesToRadians(-120))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(5.10, 5.21),
                new Rotation2d(Units.degreesToRadians(-120))), // Right Position (On Reef)
            "BlueLeftTop")); // AT 20

    blueReefSidess.add(
        new ReefSide(
            new Pose2d(new Translation2d(6.971, 4), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(5.82, 3.75),
                new Rotation2d(Units.degreesToRadians(180))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(5.91, 4.09),
                new Rotation2d(Units.degreesToRadians(180))), // Right Position (On Reef)
            "BlueRight")); // AT 21
    blueReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(5.850, 1.919), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(5.0, 2.76),
                new Rotation2d(Units.degreesToRadians(120))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(5.28, 2.89),
                new Rotation2d(Units.degreesToRadians(120))), // Right Position (On Reef)
            "BlueRightBottom")); // AT 22
    return blueReefSidess;
  }
  /*RedReef AT 7 left  x: 11.34  y: 3.67
   right  x:14.44 y:4.07

   5.82 -- 11.34
   5.91 -- 14.44
  */

  private static ArrayList<ReefSide> initRedReefPoints() {
    ArrayList<ReefSide> redReefSidess = new ArrayList<ReefSide>();

    redReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(12.1755, 1.558), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(12.19, 3),
                new Rotation2d(Units.degreesToRadians(60))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(12.43, 2.85),
                new Rotation2d(Units.degreesToRadians(60))), // Right Position (On Reef)
            "RedLeftBottom")); // AT 11
    redReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(11.4344, 4.064), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(12.393, 4.27),
                new Rotation2d(Units.degreesToRadians(0))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(12.359, 3.92),
                new Rotation2d(Units.degreesToRadians(0))), // Right Position (On Reef)
            "RedLeft")); // AT 10

    redReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(12.3022, 6.355), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(12.64, 5.30),
                new Rotation2d(Units.degreesToRadians(-60))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(12.33, 5.19),
                new Rotation2d(Units.degreesToRadians(-60))), // Right Position (On Reef)
            "RedRightTop")); // AT 9

    redReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(15.2169999999999997, 6.316),
                new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(12.66, 5.28),
                new Rotation2d(Units.degreesToRadians(-120))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(13.69, 5.28),
                new Rotation2d(Units.degreesToRadians(-120))), // Right Position (On Reef)
            "RedLeftTop")); // AT 8

    redReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(16.2311, 4), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(15.082, 3.75),
                new Rotation2d(Units.degreesToRadians(180))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(15.171, 4.09),
                new Rotation2d(Units.degreesToRadians(180))), // Right Position (On Reef)
            "RedRight")); // AT 7
    redReefSidess.add(
        new ReefSide(
            new Pose2d(
                new Translation2d(15.110, 1.919), new Rotation2d(0)), // Reef Side Position (away)
            new Pose2d(
                new Translation2d(13.45, 2.76),
                new Rotation2d(Units.degreesToRadians(120))), // Left Position (On Reef)
            new Pose2d(
                new Translation2d(13.75, 2.88),
                new Rotation2d(Units.degreesToRadians(120))), // Right Position (On Reef)
            "RedRightBottom")); // AT 6
    return redReefSidess;
  }

  private void initCoralStationPoints() {
    coralStationPoints = new ArrayList<Pose2d>();
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

    List<ReefSide> reefs = blueReefSidess;
    if (pose.getTranslation().getX() > 9) {
      reefs = redReefSidess;
    }

    ReefSide closest = null;
    double distance = 100;
    if (reefs != null) {
      for (ReefSide p : reefs) {
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
