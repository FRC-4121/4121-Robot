// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ChangeSpeedCommand extends Command {
  /** Creates a new ChangeSpeedCommand. */
  public ChangeSpeedCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(swerveDriveSpeedLimiter == driveSpeed){
      swerveDriveSpeedLimiter = slowSpeed;
      for(int i = 0; i < 4; i++){
        angleLimiters[i] = slowAngleSpeed;
      }

      SmartDashboard.putBoolean("Slow Mode", true);
      
    } else{
      swerveDriveSpeedLimiter = driveSpeed;
      for(int i = 0; i < 4; i++){
        angleLimiters[i] = angleSpeed;
      }

      SmartDashboard.putBoolean("Slow Mode", false);

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
