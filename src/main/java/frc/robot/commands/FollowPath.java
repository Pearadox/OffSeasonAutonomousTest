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
    requires(Robot.driveTrain);
    this.pathName = pathName;
    this.reverse = false;
  }

  public FollowPath(String pathName, boolean reverse) {
    requires(Robot.driveTrain);
    this.pathName = pathName;
    this.reverse = reverse;
  }

  /**
   * Tries to read the path specified in the constructor. Sets both paths to null if file is not found
   */
  private void readTrajectory() {
    try {
      leftTrajectory = new Trajectory("");
      rightTrajectory = new Trajectory("");
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
    errorL = nextLeftValues.position - Robot.driveTrain.getLeftDistance();
    errorR = nextRightValues.position - Robot.driveTrain.getRightDistance();
    totalErrorL += errorL;
    totalErrorR += errorR;

    double leftOutput = 
                        kV * nextLeftValues.velocity +
                        kA * nextLeftValues.acceleration +
                        kP * errorL +
                        kI * totalErrorL +
                        kD * errorL - lastErrorL;
                        
    double rightOutput = 
                        kV * nextRightValues.velocity +
                        kA * nextRightValues.acceleration +
                        kP * errorR +
                        kI * totalErrorR +
                        kD * errorR - lastErrorL;
    
    Robot.driveTrain.drive(leftOutput, rightOutput);
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
