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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Position;
import frc.robot.commands.CoralIntakeCommand;

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
  private static final int algaeIntakeMotorCANId = 14;
  private static final int algaeRotateMotorCANId = 11;

  private SparkMax coralRotateMotor;
  private SparkMax coralIntakeMotor;
  private SparkMax algaeRotateMotor;
  private SparkMax algaeIntakeMotor;

  private AbsoluteEncoder algaeEncoder = null;
  private AbsoluteEncoder coralEncoder = null;
  private Position targetPosition = Position.ELV_off;

  private DigitalInput bottomSensor = new DigitalInput(0);
  CommandXboxController m_CommandXboxController;

  public ElevatorSubsystem(CommandXboxController commandXboxController) {
    elevatorMotor = new SparkMax(elevatorMotorCANId, MotorType.kBrushless);
    elevatorMotorFollower = new SparkMax(elevatorMotorFollowerCANId, MotorType.kBrushless);
    elevatorMotorFollower.isFollower();
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorEncoder.setPosition(0);

    coralRotateMotor = new SparkMax(coralRotateMotorCANId, MotorType.kBrushless);
    coralEncoder = coralRotateMotor.getAbsoluteEncoder();
    coralPidController = new PIDController(2, .01, 0);
    coralPidController.enableContinuousInput(0, 1);
    algaePidController = new PIDController(1.5, .0003, 0);
    algaePidController.enableContinuousInput(0, 1);
    algaeRotateMotor = new SparkMax(algaeRotateMotorCANId, MotorType.kBrushless);
    algaeIntakeMotor = new SparkMax(algaeIntakeMotorCANId, MotorType.kBrushless);
    algaeEncoder = algaeRotateMotor.getAbsoluteEncoder();
    elevatorPidController = new PIDController(.1, .001, 0);
    coralIntakeMotor = new SparkMax(coralIntakeMotorCANId, MotorType.kBrushless);
    m_CommandXboxController = commandXboxController;
  }

  @Override
  public void periodic() {
    updateElevator();
    updateAlgae();
    updateCoral();
    updateRumble();
    SmartDashboard.putNumber("coralPosition", coralEncoder.getPosition());
    SmartDashboard.putNumber("algaePosition", algaeEncoder.getPosition());
    SmartDashboard.putNumber("TargetAngle", targetPosition.angle);
    SmartDashboard.putNumber("TargetPosition", targetPosition.position);
    SmartDashboard.putNumber("ElevatorEncoder", elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("Switch", bottomSensor.get());
    SmartDashboard.putString("Position", targetPosition.toString());
  }

  private void updateElevator() {
    if (!bottomSensor.get()) {
      elevatorEncoder.setPosition(0);
    }
    if (targetPosition != Position.ELV_off) {
      elevatorPidController.setSetpoint(targetPosition.position);
      double speed = elevatorPidController.calculate(elevatorEncoder.getPosition());

      if (speed > .7) {
        if (elevatorEncoder.getPosition() < 2) {
          speed = .3;
        } else {
          speed = .7;
        }
      } else if (speed < -.2) {
        speed = -.2;
      }

      if (targetPosition == Position.ELV_4_algae) {
        if (speed > .7) {
          speed = .7;
        } else if (speed < -.7) {
          speed = -.7;
        }
      }
      SmartDashboard.putNumber("ElevatorExpected", elevatorEncoder.getPosition());
      SmartDashboard.putNumber("elevatorSpeed", speed);

      elevatorMotor.set(speed);
    } else {
      elevatorMotor.set(0);
    }
  }

  private boolean algaeMotorRunning = false;

  private double algaeIntakeSpeed = 0;

  private void updateAlgae() {
    if (targetPosition.algae != 0) {
      algaeIntakeMotor.set(algaeIntakeSpeed);
      algaePidController.setSetpoint(targetPosition.algae);
      double speed = algaePidController.calculate(algaeEncoder.getPosition());
      if (speed > .2) {
        speed = .2;
      } else if (speed < -.3) {
        speed = -.3;
      }
      if (algaeIntakeMotor.getEncoder().getVelocity() > 0) {
        algaeMotorRunning = true;
      }
      SmartDashboard.putNumber("algaeIntakeSpeed", algaeIntakeMotor.getEncoder().getVelocity());
      if (elevatorEncoder.getPosition() > 5) {
        algaeRotateMotor.set(-speed);
      }

      /*if (algaeMotorRunning
          && algaeIntakeMotor.getEncoder().getVelocity() < 1
          && targetPosition != Position.Algae_Spit
          && algaeTimer.hasElapsed(2)) {
        // setPosition(Position.ELV_4);
        // algaeMotorRunning = false;
      }*/
    } else {
      algaeRotateMotor.set(0);
      algaeIntakeMotor.set(0);
    }
  }

  private void updateCoral() {
    if (targetPosition.angle != 0) {
      coralPidController.setSetpoint(targetPosition.angle);
      double speed = coralPidController.calculate(coralEncoder.getPosition());
      SmartDashboard.putNumber("coralSpeed", speed);
      if (speed > .5) {
        speed = .5;
      } else if (speed < -.2) {
        speed = -.2;
      }
      SmartDashboard.putString(
          "STUFF!!!!",
          targetPosition.angle + " -  " + coralEncoder.getPosition() + " _ speed = " + speed);
      coralRotateMotor.set(-speed);
    } else {
      coralRotateMotor.set(0);
    }
  }

  public void reset() {
    targetPosition = Position.ELV_off;
    coralPidController = new PIDController(2, .01, 0);
    coralIntakeMotor.set(0);
    algaeIntakeMotor.set(0);
    CoralIntakeCommand.interrupt = true;
  }

  Timer rumbleTimer = null;

  public void updateRumble() {
    if (rumbleTimer != null) {
      if (!rumbleTimer.hasElapsed(.5)) {
        m_CommandXboxController.getHID().setRumble(RumbleType.kBothRumble, 1);
      } else {
        rumbleTimer = null;
        m_CommandXboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
    }
  }

  public void setRumble() {
    rumbleTimer = new Timer();
    rumbleTimer.start();
  }

  Timer algaeTimer = null;

  public void setPosition(Position p) {
    targetPosition = p;
    if (p == Position.ELV_4_algaeLow || p == Position.ELV_4_algae) {
      algaeTimer = new Timer();
      algaeTimer.start();
      algaeIntakeSpeed = -1;
    }
  }

  public void coralOut() {
    coralIntakeMotor.set(-.6);
  }

  public void coralIn() {
    coralIntakeMotor.set(.8);
  }

  public void coralTravel() {
    coralIntakeMotor.set(.3);
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

  public double getCoralCurrent() {
    return coralIntakeMotor.getOutputCurrent();
  }

  public double getCoralVelocity() {
    return coralIntakeMotor.getEncoder().getVelocity();
  }

  public double getAlgaeVelocity() {
    return algaeIntakeMotor.getEncoder().getVelocity();
  }

  public void setSpeed(double speed) {
    algaeRotateMotor.set(speed);
  }

  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();
  }

  public void setAlgaeSpeed(double speed) {
    algaeIntakeSpeed = speed;
  }
}
