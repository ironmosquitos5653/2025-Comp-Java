// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
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
  private RelativeEncoder climbeencoder;
  private double targetAngle = .37;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climb = new SparkMax(climbMotorCANId, MotorType.kBrushless);
    rotate = new SparkMax(rotateMotorCANId, MotorType.kBrushless);
    rotateencoder = rotate.getAbsoluteEncoder();
    climbeencoder = climb.getEncoder();
    pidController = new PIDController(1, .0003, 0);
    pidController.enableContinuousInput(0, 1);
    climbeencoder.setPosition(0);
  }

  @Override
  public void periodic() {

    if (targetAngle != 0) {
      pidController.setSetpoint(targetAngle);
      double speed = pidController.calculate(rotateencoder.getPosition());
      if (speed > .2) {
        speed = .2;
      } else if (speed < -.3) {
        speed = -.3;
      }
      SmartDashboard.putNumber("climbRotateSpeed", speed);
      rotate.set(speed);
    } else {
      climb.set(0);
      climb.set(0);
    }
    SmartDashboard.putNumber("ClimbMotor", climbeencoder.getPosition());
  }

  public void setSpeed(double speed) {
    climb.set(speed);
  }

  public void armOut() {
    targetAngle = .175;
  }

  public void armIn() {
    climb.set(1);
  }

  public double getPosition() {
    return climbeencoder.getPosition();
  }

  public void reset() {
    climbeencoder.setPosition(0);
  }
}
