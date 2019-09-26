/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

enum DriveType {
  ARCADE,
  CURVATURE;
}

public class JoystickDrive extends Command {
  private DriveType driveType = DriveType.ARCADE;

  public JoystickDrive() {
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    driveType = Robot.oi.joystick.getTrigger() ? DriveType.CURVATURE : DriveType.ARCADE;
    double throttle = -Robot.oi.joystick.getY();
    double rotation = Robot.oi.joystick.getZ();
    // Set small deadband
    if (Math.abs(throttle) < .1) { throttle = 0; }
    if (Math.abs(rotation) < .1) { rotation = 0; }

    if (driveType == DriveType.ARCADE) {
      rotation /= 3;
      Robot.driveTrain.arcadeDrive(throttle, rotation);
    } else if (driveType == DriveType.CURVATURE) {
      rotation /= 5;
      Robot.driveTrain.curvatureDrive(throttle, rotation);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
