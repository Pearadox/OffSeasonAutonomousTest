/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
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

  /**
   * @return the frontLeftEncoder
   */
  public CANEncoder getFrontLeftEncoder() {
    return frontLeftEncoder;
  }

  /**
   * @return the frontRightEncoder
   */
  public CANEncoder getFrontRightEncoder() {
    return frontRightEncoder;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
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
}
