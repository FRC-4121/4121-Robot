// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveWPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import static frc.robot.Constants.*;
import edu.wpi.first.math.MathUtil;



public class FieldDriveWithJoysticks extends Command {

  private SwerveDriveWPI swervedrive;
  private XboxController Xbox;

  private final SlewRateLimiter xSpeedLimiter;
  private final SlewRateLimiter ySpeedLimiter;
  private final SlewRateLimiter rotSpeedLimiter;


  public FieldDriveWithJoysticks(SwerveDriveWPI drive, XboxController xbox) {

    swervedrive = drive;
    Xbox = xbox;

    xSpeedLimiter = new SlewRateLimiter(3);
    ySpeedLimiter = new SlewRateLimiter(3);
    rotSpeedLimiter = new SlewRateLimiter(3);

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

   final double xSpeed = xSpeedLimiter.calculate(MathUtil.applyDeadband(Xbox.getLeftX(),0.02)) * kJoystickSpeedCorr;
   final double ySpeed = ySpeedLimiter.calculate(MathUtil.applyDeadband(Xbox.getLeftY(),0.02)) * kJoystickSpeedCorr;
   final double rotSpeed = rotSpeedLimiter.calculate(MathUtil.applyDeadband(Xbox.getRightX(),0.02)) * kJoystickSpeedCorr;

    // Drive using xbox joystick values
    // kSpeedCorrection is to slow down the right motors because left motors were
    // running slower
    swervedrive.drive(xSpeed, ySpeed, rotSpeed);

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

