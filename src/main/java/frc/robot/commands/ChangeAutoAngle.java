// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.AutoAngleToTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ChangeAutoAngle extends Command {

  /** Creates a new ChangeDriveMode. */
  public ChangeAutoAngle() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (AutoAngleToTarget) {

      AutoAngleToTarget = false;

      SmartDashboard.putBoolean("AutoAngleToTarget", false);

    }
    else {

      AutoAngleToTarget = true;

      SmartDashboard.putBoolean("AutoAngleToTarget", true);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}