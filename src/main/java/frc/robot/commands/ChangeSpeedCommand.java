// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.isSlowMode;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ChangeSpeedCommand extends Command {
  /** Creates a new ChangeSpeedCommand. */
  public ChangeSpeedCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Slow Mode for WPI Swerve Drive
    if (!isSlowMode) {
      DriveConstants.LinearSpeed = DriveConstants.SlowMaxLinearSpeed;
      DriveConstants.RotationalSpeed = DriveConstants.SlowRadiansPerSecond;
      DriveConstants.maxYawRate = DriveConstants.SlowMaxYawRate;
      isSlowMode = true;
      SmartDashboard.putBoolean("Slow Mode", true);
      System.out.println("Slow Mode");
    } else {
      DriveConstants.LinearSpeed = DriveConstants.MaxLinearSpeed;
      DriveConstants.RotationalSpeed = DriveConstants.MaxRadiansPerSecond;
      DriveConstants.maxYawRate = DriveConstants.FastMaxYawRate;
      isSlowMode = false;
      SmartDashboard.putBoolean("Slow Mode", false);
      System.out.println("Fast Mode");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
