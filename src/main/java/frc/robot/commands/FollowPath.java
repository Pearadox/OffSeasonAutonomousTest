/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.FileReader;
import java.io.BufferedReader;
import java.io.File;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

import java.io.IOException;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.*;

import frc.robot.types.Trajectory;

public class FollowPath extends Command {

  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;
  
  private final String pathName;
  private final boolean reverse;
  private final boolean mirror;
  private Trajectory rightTrajectory;
  private Trajectory leftTrajectory;

  private final double kV = 1; // Velocity
  private final double kA = 1; // Acceleration
  private final double kP = 0; // Proportional
  private final double kI = 0; // Integral
  private final double kD = 0; // Derivative

  private double errorL;
  private double errorR;
  private double totalErrorL = 0d;
  private double totalErrorR = 0d;
  private double lastErrorL;
  private double lastErrorR;

  private Trajectory.PathPoint nextLeftValues;
  private Trajectory.PathPoint nextRightValues;
  
  public FollowPath(String pathName) {
    this(pathName, new char[0]);
  }

  public FollowPath(String pathName, char[] args) {
    requires(Robot.driveTrain);
    this.pathName = pathName;
    reverse = args.toString().contains("r");
    mirror = args.toString().contains("m");
  }

  /**
   * Tries to read the path specified in the constructor. Sets both paths to null if file is not found
   */
  private void readTrajectory() {
    try {
      File leftFile = new File(Filesystem.getDeployDirectory() + pathName + "_left.csv");
      File rightFile = new File(Filesystem.getDeployDirectory() + pathName + "_right.csv");
      leftTrajectory = (mirror^reverse) ? new Trajectory(rightFile) : new Trajectory(leftFile);
      rightTrajectory = (mirror^reverse) ? new Trajectory(leftFile) : new Trajectory(rightFile);
    } catch (IOException exc) {
      exc.printStackTrace();
    }
  }



  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    readTrajectory();
  } 

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    nextLeftValues = leftTrajectory.getNext();
    nextRightValues = leftTrajectory.getNext();
    errorL = (!reverse ? nextLeftValues.position : -nextLeftValues.position) - Robot.driveTrain.getLeftDistance();
    errorR = (!reverse ? nextRightValues.position : -nextLeftValues.position) - Robot.driveTrain.getRightDistance();
    totalErrorL += errorL;
    totalErrorR += errorR;

    double leftOutput = reverse ?
                        kV * nextLeftValues.velocity +
                        kA * nextLeftValues.acceleration +
                        kP * errorL +
                        kI * totalErrorL +
                        kD * errorL - lastErrorL
                        :
                        -kV * nextLeftValues.velocity +
                        -kA * nextLeftValues.acceleration +
                        kP * errorL +
                        kI * totalErrorL +
                        kD * errorL - lastErrorL;
                        
    double rightOutput = reverse ?
                        kV * nextRightValues.velocity +
                        kA * nextRightValues.acceleration +
                        kP * errorR +
                        kI * totalErrorR +
                        kD * errorR - lastErrorR
                        :
                        -kV * nextRightValues.velocity +
                        -kA * nextRightValues.acceleration +
                        kP * errorR +
                        kI * totalErrorR +
                        kD * errorR - lastErrorR;
    
    Robot.driveTrain.drive(leftOutput, rightOutput);

    lastErrorL = errorL;
    lastErrorR = errorR;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !leftTrajectory.hasNext() || !rightTrajectory.hasNext();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
