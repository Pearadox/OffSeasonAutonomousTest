/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.lib.logging.CSVLogger;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.*;

import java.io.IOException;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;
  public static DriveTrain driveTrain;
  public static AHRS gyro;
  public static Preferences prefs;
  private static ScheduledExecutorService localizationService;
  private final Lock positionLock = new ReentrantLock();
  Command autonomousCommand;
  CSVLogger logger;

  private double lastPosL;
  private double lastPosR;
  private double posX;
  private double posY;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    driveTrain = new DriveTrain();
    oi = new OI();
    gyro = new AHRS();
    prefs = Preferences.getInstance();
    localizationService = Executors.newSingleThreadScheduledExecutor();
    localizationService.scheduleWithFixedDelay(this::updatePosition, 0, 5, TimeUnit.MILLISECONDS);

    autonomousCommand = new FollowPath("Test", "");

    driveTrain.setEncoders();
    // chooser.addOption("My Auto", new MyAutoCommand());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    logger.stop();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    newLogger();
    if (logger != null) {
      logger.start();
    }
    if (autonomousCommand != null) { 
      autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (logger == null) {
      newLogger();
    }
    if (autonomousCommand.isRunning()) { 
      autonomousCommand.cancel(); 
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private void newLogger() {
    try {
      logger = new CSVLogger();
    } catch (IOException e) {
      System.out.println(e.getMessage());
    }
  }

  private synchronized void updatePosition() {
    double changeInPos = (driveTrain.getLeftDistance() - lastPosL
        + driveTrain.getRightDistance() - lastPosR) / 2;
    double angle = Math.toRadians(gyro.getYaw());
    setPos(posX + changeInPos * Math.cos(angle), posY + changeInPos * Math.sin(angle));
  }

  private void setPos(double x, double y) {
    positionLock.lock();
    try {
      posX = x;
      posY = y;
    } finally {
      positionLock.unlock();
    }
  }
}
