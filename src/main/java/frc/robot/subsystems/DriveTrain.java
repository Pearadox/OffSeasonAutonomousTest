/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.JoystickDrive;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private SpeedController frontLeft = new CANSparkMax(RobotMap.FRONT_LEFT_MOTOR_ID, MotorType.kBrushless);
  private SpeedController frontRight = new CANSparkMax(RobotMap.FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);
  private SpeedController backLeft = new CANSparkMax(RobotMap.BACK_LEFT_MOTOR_ID, MotorType.kBrushless);
  private SpeedController backRight = new CANSparkMax(RobotMap.BACK_RIGHT_MOTOR_ID, MotorType.kBrushless);

  private CANEncoder frontLeftEncoder = new CANEncoder((CANSparkMax) frontLeft);
  private CANEncoder frontRightEncoder = new CANEncoder((CANSparkMax) frontRight);
  private CANEncoder backLeftEncoder = new CANEncoder((CANSparkMax) backLeft);
  private CANEncoder backRightEncoder = new CANEncoder((CANSparkMax) backRight);

  private double gearboxRatio = 4.67;
  private double shaftWheelRatio = 26d/12d;
  private double wheelCircumfrence = Math.PI / 2;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new JoystickDrive());
  }

  public void drive(double leftSpeed, double rightSpeed) {
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
  }

  private void setLeftSpeed(double speed) {
    frontLeft.set(speed);
    backLeft.set(speed);
  }

  private void setRightSpeed(double speed) {
    frontRight.set(speed);
    backRight.set(speed);
  }

  /**************************************************************
   * Unless otherwise noted, all values are in feet and seconds *
   **************************************************************/
  public double getLeftPosition() {
    return (frontLeftEncoder.getPosition() + backLeftEncoder.getPosition())/2;
  }

  public double getRightPosition() {
    return (frontRightEncoder.getPosition() + backRightEncoder.getPosition())/2;
  }

  public double getLeftDistance() {
    return getLeftPosition() * wheelCircumfrence / (gearboxRatio * shaftWheelRatio);
  }

  public double getRightDistance() {
    return getLeftPosition() * wheelCircumfrence / (gearboxRatio * shaftWheelRatio);
  }
}
