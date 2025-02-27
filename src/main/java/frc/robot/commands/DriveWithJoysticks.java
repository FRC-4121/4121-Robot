// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.*;



public class DriveWithJoysticks extends Command {

  private Drivetrain tankDrive;
  private XboxController Xbox;

 

  public DriveWithJoysticks(Drivetrain drive, XboxController xbox) {

    tankDrive = drive;
    Xbox = xbox;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tankDrive);
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
    tankDrive.drive(Xbox.getLeftY(), Xbox.getRightY());

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
