// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private static final int climbMotorCANId = 32;
  private static final int rotateMotorCANId = 30;
  private SparkMax climb;
  private SparkMax rotate;

  private PIDController pidController;

  private AbsoluteEncoder rotateencoder = null;

  private double targetAngle = .321;
  private boolean enabled = true;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climb = new SparkMax(climbMotorCANId, MotorType.kBrushless);
    rotate = new SparkMax(rotateMotorCANId, MotorType.kBrushless);
    rotateencoder = rotate.getAbsoluteEncoder();

    pidController = new PIDController(5, 0, 0.3);
    pidController.enableContinuousInput(0, 1);
  }

  @Override
  public void periodic() {

    if (targetAngle != 0 && enabled) {
      pidController.setSetpoint(targetAngle);
      double speed = pidController.calculate(rotateencoder.getPosition());
      if (speed > .1) {
        speed = .1;
      } else if (speed < -.5) {
        speed = -.1;
      }
      SmartDashboard.putNumber("climbRotateSpeed", speed);
      rotate.set(speed);
    } else {
      rotate.set(0);
    }
  }

  public void setSpeed(double speed) {
    climb.set(speed);
  }

  public void armOut() {
    targetAngle = .17;
  }

  public void enableClimber(boolean e) {
    enabled = e;
  }
}
