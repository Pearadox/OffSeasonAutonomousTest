/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private SpeedController frontLeft = new CANSparkMax(RobotMap.FRONT_LEFT_MOTOR_ID,
      MotorType.kBrushless);
  private SpeedController frontRight = new CANSparkMax(RobotMap.FRONT_RIGHT_MOTOR_ID,
      MotorType.kBrushless);
  private SpeedController backLeft = new CANSparkMax(RobotMap.BACK_LEFT_MOTOR_ID,
      MotorType.kBrushless);
  private SpeedController backRight = new CANSparkMax(RobotMap.BACK_RIGHT_MOTOR_ID,
      MotorType.kBrushless);

  private CANEncoder frontLeftEncoder = new CANEncoder((CANSparkMax) frontLeft);
  private CANEncoder frontRightEncoder = new CANEncoder((CANSparkMax) frontRight);
  private CANEncoder backLeftEncoder = new CANEncoder((CANSparkMax) backLeft);
  private CANEncoder backRightEncoder = new CANEncoder((CANSparkMax) backRight);

  private static final double GEARBOX_RATIO = 4.67;
  private static final double SHAFT_WHEEL_RATIO = 26d / 12d;
  private static final double WHEEL_CIRCUMFRENCE = Math.PI / 2;

  private static final double QUICK_TURN_THRESHOLD = .1;
  private static final double DEADBAND = .1;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new JoystickDrive());
  }

  public void setEncoders(double position) {
    frontLeftEncoder.setPosition(position);
    frontRightEncoder.setPosition(position);
    backLeftEncoder.setPosition(position);
    backRightEncoder.setPosition(position);
  }

  public void setEncoders() {
    setEncoders(0);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    if (Math.abs(leftSpeed) <= DEADBAND) { 
      leftSpeed = 0; 
    }
    if (Math.abs(rightSpeed) <= DEADBAND) { 
      rightSpeed = 0; 
    }
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
  }

  public void arcadeDrive(double throttle, double rotation) {
    if (Math.abs(throttle) <= DEADBAND) { 
      throttle = 0; 
    }
    if (Math.abs(rotation) <= DEADBAND) { 
      rotation = 0; 
    }
    setLeftSpeed(throttle + rotation);
    setRightSpeed(throttle - rotation);
  }

  public void curvatureDrive(double throttle, double rotation) {
    double leftOutput;
    double rightOutput;

    // Quickturn
    if (Math.abs(throttle) < QUICK_TURN_THRESHOLD) {
      leftOutput = rotation;
      rightOutput = -rotation;
    } else {
      leftOutput = throttle + throttle * rotation;
      rightOutput = throttle - throttle * rotation;

      if (leftOutput > 1) {
        rightOutput -= leftOutput - 1;
        leftOutput = 1;
      } else if (rightOutput > 1) {
        leftOutput -= rightOutput - 1;
        rightOutput = 1;
      } else if (leftOutput < -1) {
        rightOutput -= leftOutput + 1;
        leftOutput = -1;
      } else if (rightOutput > -1) {
        leftOutput -= rightOutput + 1;
        rightOutput = 1;
      }
    }

    setLeftSpeed(leftOutput);
    setRightSpeed(rightOutput);
  }

  private void setLeftSpeed(double speed) {
    frontLeft.set(speed);
    backLeft.set(speed);
  }

  private void setRightSpeed(double speed) {
    frontRight.set(speed);
    backRight.set(speed);
  }

  /*------------------------------------------------------------*/
  /* Unless otherwise noted, all values are in feet and seconds */
  /*------------------------------------------------------------*/

  public double getLeftPosition() {
    return (frontLeftEncoder.getPosition() + backLeftEncoder.getPosition()) / 2;
  }

  public double getRightPosition() {
    return (frontRightEncoder.getPosition() + backRightEncoder.getPosition()) / 2;
  }

  public double getLeftDistance() {
    return getLeftPosition() * WHEEL_CIRCUMFRENCE / (GEARBOX_RATIO * SHAFT_WHEEL_RATIO);
  }

  public double getRightDistance() {
    return getLeftPosition() * WHEEL_CIRCUMFRENCE / (GEARBOX_RATIO * SHAFT_WHEEL_RATIO);
  }

}
