// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveWPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.controller.*;

import edu.wpi.first.math.MathUtil;

public class DriveWithJoysticks extends Command {
  private SwerveDriveWPI swervedrive;
  private XboxController Xbox;
  private NetworkTableQuerier table;

  private final SlewRateLimiter xSpeedLimiter;
  private final SlewRateLimiter ySpeedLimiter;
  private final SlewRateLimiter rotSpeedLimiter;

  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  private double tagsFound;
  private int centerTag;
  private int tagID;
  private double tagDistance;
  private double tagOffset;
  private Boolean isMyTag;

  private MedianFilter distFilter;
  private MedianFilter offsetFilter;

  private PIDController wpiPIDController;

  public DriveWithJoysticks(SwerveDriveWPI drive, XboxController xbox, NetworkTableQuerier ntable) {

    swervedrive = drive;
    Xbox = xbox;
    table = ntable;

    xSpeedLimiter = new SlewRateLimiter(2);
    ySpeedLimiter = new SlewRateLimiter(2);
    rotSpeedLimiter = new SlewRateLimiter(3);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swervedrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    isMyTag = false;
    distFilter = new MedianFilter(10);
    offsetFilter = new MedianFilter(10);

    // Create PID controller
    wpiPIDController = new PIDController(DriveConstants.kPAutoAlign, DriveConstants.kIAutoAlign,
        DriveConstants.kDAutoAlign);
    wpiPIDController.setTolerance(1.0, 5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Initialize variables
    tagOffset = 0;
    isMyTag = false;

    // Get joystick positions and convert to speeds
    xSpeed = xSpeedLimiter.calculate(MathUtil.applyDeadband(Xbox.getLeftX(), 0.01))
        * ControlConstants.kJoystickSpeedCorr;
    ySpeed = ySpeedLimiter.calculate(MathUtil.applyDeadband(Xbox.getLeftY(), 0.01))
        * ControlConstants.kJoystickSpeedCorr;
    rotSpeed = -rotSpeedLimiter.calculate(MathUtil.applyDeadband(Xbox.getRightX(), 0.01))
        * ControlConstants.kJoystickSpeedCorr;
    System.out.println("xSpeed:" + xSpeed);
    System.out.println("ySpeed: " + ySpeed);
    System.out.println("rotSpeed: " + rotSpeed);

    // Check if auto align is enabled before proceeding
    if (DriveConstants.AutoAngleToTarget) {
      if (Math.abs(rotSpeed) < 0.01) {
        // Determine if we see any AprilTags
        tagsFound = table.getTagsFound("CAM2");
        if (tagsFound > 1) {
          // Determine if we see the center speaker tag
          centerTag = -1;
          for (int i = 0; i < tagsFound; i++) {
            if (Constants.blueAlliance) {
              if ((int) table.getTagInfo("Cam2", i, "id") == MechanismConstants.BlueSpeakerCenterID) {
                centerTag = i;
                isMyTag = true;
              }
            } else {
              if ((int) table.getTagInfo("Cam2", i, "id") == MechanismConstants.RedSpeakerCenterID) {
                centerTag = i;
                isMyTag = true;
              }
            }
          }

          // If we found the center speaker tag, calculate angle correction
          if (isMyTag) {
            // Only find distance and offset if we know we have the right tag
            tagDistance = distFilter.calculate(table.getTagInfo("CAM2", centerTag, "distance"));
            tagOffset = offsetFilter.calculate(table.getTagInfo("CAM2", centerTag, "offset"));
            if (tagDistance > DriveConstants.MinAutoAlignDistance
                && tagDistance < DriveConstants.MaxAutoAlignDistance) {
              System.out.println("Tag Offset: " + tagOffset);
              rotSpeed = -wpiPIDController.calculate(tagOffset, 0);

              if (rotSpeed > 0)
                rotSpeed = DriveConstants.kFFAutoAlign + rotSpeed;
              else if (rotSpeed < 0)
                rotSpeed = -DriveConstants.kFFAutoAlign + rotSpeed;

              if (rotSpeed > 0.4)
                rotSpeed = 0.4;
              else if (rotSpeed < -0.4)
                rotSpeed = -0.4;

              if (Math.abs(tagOffset) < 5.0)
                rotSpeed = 0.0;

              System.out.println("Auto Rotate");
              System.out.println("rotSpeed: " + rotSpeed);

            }
          }
        }
      }
    }
    if (Constants.isFieldOriented)
      swervedrive.driveFieldRelative(xSpeed, ySpeed, rotSpeed);
    else
      swervedrive.driveRobotRelative(xSpeed, ySpeed, rotSpeed);
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

}
