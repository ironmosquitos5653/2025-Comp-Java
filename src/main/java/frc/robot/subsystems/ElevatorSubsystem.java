// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Position;

public class ElevatorSubsystem extends SubsystemBase {

  private static final int elevatorMotorCANId = 16;
  private SparkMax elevatorMotor;
  private RelativeEncoder elevatorEncoder;

  private static final int elevatorMotorFollowerCANId = 10;
  private SparkMax elevatorMotorFollower;

  private PIDController elevatorPidController;
  private PIDController algaePidController;
  private PIDController coralPidController;

  private static final int coralIntakeMotorCANId = 12;
  private static final int coralRotateMotorCANId = 13;
  private static final int algaeIntakeMotorCANId = 0;
  private static final int algaeRotateMotorCANId = 11;

  private SparkMax coralRotateMotor;
  private SparkMax coralIntakeMotor;
  private SparkMax algaeRotateMotor;
  private SparkMax algaeIntakeMotor;

  private AbsoluteEncoder algaeEncoder = null;
  private AbsoluteEncoder coralEncoder = null;
  private Position targetPosition = Position.ELV_off;

  private DigitalInput bottomSensor = new DigitalInput(0);

  public ElevatorSubsystem() {
    elevatorMotor = new SparkMax(elevatorMotorCANId, MotorType.kBrushless);
    elevatorMotorFollower = new SparkMax(elevatorMotorFollowerCANId, MotorType.kBrushless);
    elevatorMotorFollower.isFollower();
    elevatorEncoder = elevatorMotor.getEncoder();
    // elevatorEncoder.setPosition(0);

    /*
    coralRotateMotor = new SparkMax(coralRotateMotorCANId, MotorType.kBrushless);
    coralIntakeMotor = new SparkMax(coralIntakeMotorCANId, MotorType.kBrushless);
    algaeRotateMotor = new SparkMax(algaeRotateMotorCANId, MotorType.kBrushless);
    algaeIntakeMotor = new SparkMax(algaeIntakeMotorCANId, MotorType.kBrushless);

    coralEncoder = coralRotateMotor.getAbsoluteEncoder();
    algaeEncoder = algaeRotateMotor.getAbsoluteEncoder();

    coralPidController = new PIDController(.015, .0003, 0);
    algaePidController = new PIDController(.015, .0003, 0);
    */
    elevatorPidController = new PIDController(.3, .001, 0);
  }

  @Override
  public void periodic() {
    updateElevator();
    // updateAlgae();
    // updateCoral();
    SmartDashboard.putNumber("ElevatorEncoder", elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("Switch", bottomSensor.get());
  }

  private void updateElevator() {
    if (!bottomSensor.get()) {
      elevatorEncoder.setPosition(0);
    }
    if (targetPosition != Position.ELV_off) {
      elevatorPidController.setSetpoint(targetPosition.position);
      double speed = elevatorPidController.calculate(elevatorEncoder.getPosition());
      if (speed > .4) {
        speed = .4;
      } else if (speed < -.1) {
        speed = -.1;
      }

      elevatorMotor.set(speed);
    } else {
      elevatorMotor.set(0);
    }
  }

  private void updateAlgae() {
    /*if (targetPosition != Position.ELV_off) {
      algaePidController.setSetpoint(targetPosition.angle);
      double speed = algaePidController.calculate(algaeEncoder.getPosition());
      if(speed > .2) {
        speed = .2;
      } else if ( speed < -.2) {
        speed = -.2;
      }

      algaeRotateMotor.set(-speed);
    } else {
      algaeRotateMotor.set(0);
    }*/
  }

  private void updateCoral() {
    if (targetPosition != Position.ELV_off) {
      coralPidController.setSetpoint(targetPosition.angle);
      double speed = coralPidController.calculate(coralEncoder.getPosition());
      if (speed > .2) {
        speed = .2;
      } else if (speed < -.2) {
        speed = -.2;
      }

      coralRotateMotor.set(-speed);
    } else {
      coralRotateMotor.set(0);
    }
  }

  public void setPosition(Position p) {
    targetPosition = p;
  }

  public void coralOut() {
    coralIntakeMotor.set(.3);
  }

  public void coralIn() {
    coralIntakeMotor.set(-.3);
  }

  public void coralOff() {
    coralIntakeMotor.set(0);
  }

  public void algaeOut() {
    algaeIntakeMotor.set(.3);
  }

  public void algaeIn() {
    algaeIntakeMotor.set(-.3);
  }

  public void algaeOff() {
    algaeIntakeMotor.set(0);
  }

  public double getCoralVelocity() {
    return coralRotateMotor.getEncoder().getVelocity();
  }

  public void setSpeed(double speed) {
    coralRotateMotor.set(speed);
  }
}
