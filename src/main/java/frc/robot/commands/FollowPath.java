/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.lib.pathfollowing.Point;
import frc.lib.pathfollowing.Trajectory;

import frc.robot.Robot;
import frc.robot.RobotMap;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class FollowPath extends Command {

  private final String pathName;
  private final boolean reverse;
  private final boolean mirror;
  private Trajectory rightTrajectory;
  private Trajectory leftTrajectory;
  private final double initialHeading;
  private final double pathStartHeading;

  private static double V_PP = 1 / RobotMap.MAX_VELOCITY; // Velocity
  private static double A_PP = .035; // Acceleration
  private static double H_PP = -.009; // Heading
  private static double P = .12; // Proportional
  private static double I = 0; // Integral
  private static double D = 0; // Derivative

  // Variables used in execute, declared here to avoid GC
  private double errorL;
  private double errorR;
  private double totalErrorL = 0d;
  private double totalErrorR = 0d;
  private double lastErrorL;
  private double lastErrorR;
  private double errorH;

  private double startTime;

  private Point nextLeftValues;
  private Point nextRightValues;

  public FollowPath(String pathName) {
    this(pathName, new char[0]);
  }

  /**
   * Follows a path.
   * 
   * @param pathName Name of the path in filesystem
   * @param args     Character array of arguments: - 'R' or 'r' for reversed - 'M'
   *                 or 'm' for mirrored
   */
  public FollowPath(String pathName, char[] args) {
    requires(Robot.driveTrain);
    this.pathName = pathName;
    reverse = args.toString().contains("r") || args.toString().contains("R");
    mirror = args.toString().contains("m") || args.toString().contains("M");
    initialHeading = boundTo180(Robot.gyro.getYaw());
    startTime = Timer.getFPGATimestamp();
    pathStartHeading = leftTrajectory.first().heading;
  }

  /**
   * Follows a Path.
   * 
   * @param pathName Name of the path in filesystem
   * @param args     String of arguments: - 'R' or 'r' for reversed - 'M' or 'm'
   *                 for mirrored
   */
  public FollowPath(String pathName, String args) {
    this(pathName, args.toCharArray());
  }

  private void readTrajectory() {
    try {
      File leftFile = new File(Filesystem.getDeployDirectory() 
          + "paths/" + pathName + "_left.csv");
      File rightFile = new File(Filesystem.getDeployDirectory() 
          + "paths/" + pathName + "_right.csv");
      leftTrajectory = (mirror ^ reverse) ? new Trajectory(rightFile) : new Trajectory(leftFile);
      rightTrajectory = (mirror ^ reverse) ? new Trajectory(leftFile) : new Trajectory(rightFile);
    } catch (IOException exc) {
      exc.printStackTrace();
      leftTrajectory = null;
      rightTrajectory = null;
    }
  }

  private void shuffleboardSetup() {
    Robot.prefs = Preferences.getInstance();

    if (!Preferences.getInstance().containsKey("kV")) {
      Preferences.getInstance().putDouble("kV", V_PP);
    }
    if (!Preferences.getInstance().containsKey("kH")) {
      Preferences.getInstance().putDouble("kH", H_PP);
    }
    if (!Preferences.getInstance().containsKey("kA")) {
      Preferences.getInstance().putDouble("kA", A_PP);
    }
    if (!Preferences.getInstance().containsKey("kP")) {
      Preferences.getInstance().putDouble("kP", P);
    }
    if (!Preferences.getInstance().containsKey("kI")) {
      Preferences.getInstance().putDouble("kI", I);
    }
    if (!Preferences.getInstance().containsKey("kD")) {
      Preferences.getInstance().putDouble("kD", D);
    }

    V_PP = Preferences.getInstance().getDouble("kV", V_PP);
    H_PP = Preferences.getInstance().getDouble("kH", H_PP);
    A_PP = Preferences.getInstance().getDouble("kA", A_PP);
    P = Preferences.getInstance().getDouble("kP", P);
    I = Preferences.getInstance().getDouble("kI", I);
    D = Preferences.getInstance().getDouble("kD", D);
  }

  private double boundTo180(double degrees) {
    while (degrees > 180 || degrees < -180) {
      if (degrees > 180) {
        degrees -= 360;
      }
      if (degrees < -180) {
        degrees += 360;
      }
    }
    return degrees;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    readTrajectory();
    shuffleboardSetup();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    nextLeftValues = leftTrajectory.next();
    nextRightValues = rightTrajectory.next();
    if (reverse) {
      nextLeftValues.position *= -1;
      nextLeftValues.acceleration *= -1;
      nextLeftValues.velocity *= -1;
      nextRightValues.position *= -1;
      nextRightValues.acceleration *= -1;
      nextRightValues.velocity *= -1;
    }
    if (reverse ^ mirror) {
      errorL = nextLeftValues.position - Robot.driveTrain.getRightDistance();
      errorR = nextRightValues.position - Robot.driveTrain.getLeftDistance();
    } else {
      errorL = nextLeftValues.position - Robot.driveTrain.getLeftDistance();
      errorR = nextRightValues.position - Robot.driveTrain.getRightDistance();
    }
    totalErrorL += errorL;
    totalErrorR += errorR;
    errorH = (nextLeftValues.heading - pathStartHeading) 
      - (boundTo180(Robot.gyro.getYaw()) - initialHeading);

    double leftOutput = V_PP * nextLeftValues.velocity 
        + A_PP * nextLeftValues.acceleration 
        + P * errorL
        + I * totalErrorL 
        + D * (errorL - lastErrorL) 
        + H_PP * errorH;

    double rightOutput = V_PP * nextRightValues.velocity 
        + A_PP * nextRightValues.acceleration 
        + P * errorR
        + I * totalErrorR 
        + D * (errorR - lastErrorR) 
        - H_PP * errorH;

    Robot.driveTrain.tankDrive(leftOutput, rightOutput);

    lastErrorL = errorL;
    lastErrorR = errorR;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (!leftTrajectory.hasNext() 
        || !rightTrajectory.hasNext() 
        || leftTrajectory == null 
        || rightTrajectory == null
        || Timer.getFPGATimestamp() - startTime >= leftTrajectory.getTotalTime()) {
      return true;
    }
    return false;
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
