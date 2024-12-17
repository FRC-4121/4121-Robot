// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.cameras.CameraBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    new CameraBuilder(0, "Camera 0").fps(15).attachAutoCapture();
    new CameraBuilder(1, "Camera 1").fps(15).attachAutoCapture();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Put zero mechanism options on the dashboard
    SmartDashboard.putNumber("Zero Gyro", 0);
    SmartDashboard.putNumber("Zero Positions", 0);
    SmartDashboard.putNumber("Zero Encoder", 0);

    // Put auto program notes to shoot on the dashboard
    SmartDashboard.putNumber("Auto Notes", 2);

    // Put auto position on the dashboard
    SmartDashboard.putNumber("Auto Position", 1);

    // Get the alliance color
    m_robotContainer.getAllianceColor();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Keep checking alliance color
    m_robotContainer.getAllianceColor();

    // Get auto position
    m_robotContainer.getAutoPosition();

    // Get park selection
    m_robotContainer.getParkSelection();

    // Get color selection
    m_robotContainer.getColorSelection();

    // Get shooter selection
    m_robotContainer.getAngleSelection();

    // Get angle to target selection
    m_robotContainer.getAngleToTargetSelection();

    // Check for note on board
    m_robotContainer.checkForNote();

    // Update Robot Status
    m_robotContainer.updateRobotStatus();

    // Check for robot zero command and zero the robot
    if (SmartDashboard.getNumber("Zero Positions", 0) == 1) {
      m_robotContainer.zeroRobot();
      SmartDashboard.putNumber("Zero Positions", 0);
    }

    // Check for robot zero command and zero the robot
    if (SmartDashboard.getNumber("Zero Encoder", 0) == 1) {
      m_robotContainer.zeroDriveEncoder();
      SmartDashboard.putNumber("Zero Encoder", 0);
    }

    // Check for number of notes to shoot in auto
    Constants.autoNotes = (int) SmartDashboard.getNumber("Auto Notes", 2);

    // Check current speed setting and update dashboard
    if (DriveConstants.LinearSpeed == DriveConstants.MaxLinearSpeed) {
      SmartDashboard.putBoolean("Slow Mode", false);
    } else {
      SmartDashboard.putBoolean("Slow Mode", true);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

    // m_robotContainer.stopPi();

  }

  @Override
  public void disabledPeriodic() {

    // m_robotContainer.stopPi();

  }

  @Override
  public void teleopExit() {

    // m_robotContainer.stopPi();

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    // Get the autonomous command
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // Zero the shooter angle encoder
    m_robotContainer.zeroRobot();

    // Set robot to robot oriented driving
    Constants.isFieldOriented = true;

    // Make sure that the Auto Shooter Command will run
    MechanismConstants.AutoShooterPositioning = true;

    // Make sure the robot is in fast mode
    DriveConstants.LinearSpeed = DriveConstants.MaxLinearSpeed;
    DriveConstants.RotationalSpeed = DriveConstants.MaxRadiansPerSecond;
    DriveConstants.maxYawRate = DriveConstants.FastMaxYawRate;
    Constants.isSlowMode = false;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Turn on auto shooter speed control
    Constants.runAutoSpeedControl = true;

    // Set robot to field oriented driving
    Constants.isFieldOriented = true;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // get Arm Selection
    // m_robotContainer.getArmSelection();

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
