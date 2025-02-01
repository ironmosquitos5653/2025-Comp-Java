// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private static final int leftMotorCANId = 16;
  private SparkMax leftMotor;

  private static final int rightMotorCANId = 0;
  private SparkMax rightMotor;

  private PIDController pidController;

  private double targetAngle = 0;

  public ElevatorSubsystem() {
    leftMotor = new SparkMax(leftMotorCANId, MotorType.kBrushless);
    rightMotor = new SparkMax(rightMotorCANId, MotorType.kBrushless);

    pidController = new PIDController(.015, .0003, 0);
  }

  @Override
  public void periodic() {}
}
