/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

import java.io.IOException;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.*;
import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;

public class FollowPath extends Command {

  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder; 
  
  private final String pathName;
  private Trajectory leftTrajectory;
  private Trajectory rightTrajectory;

  private EncoderFollower leftFollower;
  private EncoderFollower rightFollower;

  private Notifier followerNotifier;
  
  public FollowPath(String pathName) {
    requires(Robot.driveTrain);
    this.pathName = pathName;
  }

  /**
   * Tries to read the path specified in the constructor. Sets both paths to null if file is not found
   */
  private void readTrajectory() {
    try {
      // Paths switched due to PathWeaver bug
      this.leftTrajectory = PathfinderFRC.getTrajectory(this.pathName + ".right");
      this.rightTrajectory = PathfinderFRC.getTrajectory(this.pathName + ".left");
    } catch (IOException e) {
      System.out.printf("Path %s not found", this.pathName);
      leftTrajectory = null;
      rightTrajectory = null;
      return;
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    readTrajectory();

    leftEncoder = Robot.driveTrain.getFrontLeftEncoder();
    rightEncoder = Robot.driveTrain.getFrontRightEncoder();

    leftFollower = new EncoderFollower(leftTrajectory);
    rightFollower = new EncoderFollower(rightTrajectory);

    // TODO: Tune PID Values

    final double P = 1d;
    final double I = 0d;
    final double D = 0d;

    leftFollower.configurePIDVA(P, I, D, 1 / RobotMap.MAX_VELOCITY, 0);
    rightFollower.configurePIDVA(P, I, D, 1 / RobotMap.MAX_VELOCITY, 0);

    int leftCount = (int) leftEncoder.getPosition() / leftEncoder.getCPR();
    int rightCount = (int) rightEncoder.getPosition() / rightEncoder.getCPR();

    leftFollower.configureEncoder(leftCount, leftEncoder.getCPR(), RobotMap.WHEEL_DIAMETER);
    rightFollower.configureEncoder(rightCount, rightEncoder.getCPR(), RobotMap.WHEEL_DIAMETER);
    
    followerNotifier = new Notifier(this::execute);
  } 

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    int leftCount = (int) leftEncoder.getPosition() / leftEncoder.getCPR();
    int rightCount = (int) rightEncoder.getPosition() / rightEncoder.getCPR();
    double leftSpeed = leftFollower.calculate(leftCount);
    double rightSpeed = leftFollower.calculate(rightCount);
    double heading = Robot.gyro.getAngle();
    double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
    double headingDifference = Pathfinder.boundHalfDegrees(desiredHeading - heading);
    double turn = 0.8 * (-1d / 80d) * headingDifference;

    Robot.driveTrain.drive(leftSpeed + turn, rightSpeed - turn);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (leftTrajectory == null || rightTrajectory == null) { return true; } // Invalid path
    if (leftFollower.isFinished() || rightFollower.isFinished()) { return true; } // Finished path
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    followerNotifier.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
