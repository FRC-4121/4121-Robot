// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDriveWPI;
import frc.robot.ExtraClasses.*;


public class AutoAlignToReef extends Command {

  // Declare local variables
  private SwerveDriveWPI swerveDrive;
  private NetworkTableQuerier ntables;

  /** Creates a new AutoAlignToReef. */
  public AutoAlignToReef(SwerveDriveWPI swerve, NetworkTableQuerier tables) {

    // Set local variables
    swerveDrive = swerve;
    ntables = tables;

    // Declare subsystem requirements
    addRequirements(swerveDrive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
