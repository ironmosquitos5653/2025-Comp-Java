// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private static final int climbMotorCANId = 0;
  private SparkMax climb;

  private PIDController pidController;

  private AbsoluteEncoder encoder = null;
  private double targetAngle = 0;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climb = new SparkMax(climbMotorCANId, MotorType.kBrushless);
    encoder = climb.getAbsoluteEncoder();
    pidController = new PIDController(.015, .0003, 0);
  }

  public ClimbSubsystem(String getSubsystem) {
    // TODO Auto-generated constructor stub
  }

  public void toggle() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
