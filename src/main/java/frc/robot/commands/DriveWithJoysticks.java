// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import edu.wpi.first.math.controller.*;

import edu.wpi.first.math.MathUtil;

public class DriveWithJoysticks extends Command {
  private SwerveDrive swerve;
  private XboxController xbox;

  private final SlewRateLimiter xSpeedLimiter;
  private final SlewRateLimiter ySpeedLimiter;
  private final SlewRateLimiter rotSpeedLimiter;

  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  private PIDController wpiPIDController;

  private boolean isFieldOriented;

  public DriveWithJoysticks(SwerveDrive swerve, XboxController xbox, NetworkTableQuerier ntable) {

    this.swerve = swerve;
    this.xbox = xbox;

    xSpeedLimiter = new SlewRateLimiter(2);
    ySpeedLimiter = new SlewRateLimiter(2);
    rotSpeedLimiter = new SlewRateLimiter(3);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Create PID controller
    wpiPIDController = new PIDController(DriveConstants.kPAutoAlign, DriveConstants.kIAutoAlign,
        DriveConstants.kDAutoAlign);
    wpiPIDController.setTolerance(1.0, 5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get joystick positions and convert to speeds
    xSpeed = xSpeedLimiter.calculate(MathUtil.applyDeadband(xbox.getLeftX(), 0.01))
        * ControlConstants.kJoystickSpeedCorr;
    ySpeed = ySpeedLimiter.calculate(MathUtil.applyDeadband(xbox.getLeftY(), 0.01))
        * ControlConstants.kJoystickSpeedCorr;
    rotSpeed = -rotSpeedLimiter.calculate(MathUtil.applyDeadband(xbox.getRightX(), 0.01))
        * ControlConstants.kJoystickSpeedCorr;
    System.out.println("xSpeed:" + xSpeed);
    System.out.println("ySpeed: " + ySpeed);
    System.out.println("rotSpeed: " + rotSpeed);

    // Check if auto align is enabled before proceeding
    if (DriveConstants.AutoAngleToTarget) {
      if (Math.abs(rotSpeed) < 0.01) {

      }
    }
    if (Constants.isFieldOriented)
      swerve.driveFieldRelative(xSpeed, ySpeed, rotSpeed);
    else
      swerve.driveRobotRelative(xSpeed, ySpeed, rotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean getFieldOriented() {
    return isFieldOriented;
  }

  public void setFieldOriented(boolean isFieldOriented) {
    this.isFieldOriented = isFieldOriented;
  }
}
