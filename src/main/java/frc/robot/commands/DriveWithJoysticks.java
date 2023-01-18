// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import static frc.robot.Constants.SwerveDriveConstants.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveWithJoysticks extends CommandBase {

  private SwerveDrive swervedrive;
  private XboxController Xbox;


  public DriveWithJoysticks(SwerveDrive drive, XboxController xbox) {

    swervedrive = drive;
    Xbox = xbox;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swervedrive);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Drive using xbox joystick values
    // kSpeedCorrection is to slow down the right motors because left motors were
    // running slower
    swervedrive.drive(kJoystickSpeedCorr * Xbox.getLeftX(), kJoystickSpeedCorr * Xbox.getLeftY(), kJoystickSpeedCorr * Xbox.getRightX());

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
