// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
  private static final int coralRotateMotorCANId = 0;
  private static final int coralIntakeMotorCANId = 0;
  private SparkMax coralRotateMotor;
  private SparkMax coralIntakeMotor;

  private PIDController pidController;

  private AbsoluteEncoder encoder = null;
  private double targetAngle = 0;

  /** Creates a new ClimbSubsystem. */
  public CoralSubsystem() {
    coralRotateMotor = new SparkMax(coralRotateMotorCANId, MotorType.kBrushless);
    coralIntakeMotor = new SparkMax(coralIntakeMotorCANId, MotorType.kBrushless);
    encoder = coralRotateMotor.getAbsoluteEncoder();
    pidController = new PIDController(.015, .0003, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
