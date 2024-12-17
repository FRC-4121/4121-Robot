// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ControlConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveWPI;
import edu.wpi.first.math.controller.*;


public class AutoDrive extends Command {

  /** Creates a new AutoDrive. */
  private final SwerveDriveWPI drivetrain;
  private double targetDriveDistance; //inches
  private double targetAngle;
  private double targetRotation;
  private double targetSpeed;
  private double stopTime;
  private double currentGyroAngle = 0;
  private double gyroOffset;
  private double frontAngle;

  private double startTime;
  private double distanceTraveled;

  private boolean firstRun = true;

  private Timer timer = new Timer();
  private PIDController pidFrontAngle;

  //Distance is in inches, target rotation is a value from -1 to 1 or and angle? 
  public AutoDrive(SwerveDriveWPI drive, double speed, double dis, double ang, double heading, double rotation, double time) {
   
    targetSpeed = speed;
    targetDriveDistance = dis;
    targetAngle = ang; //Divide by 360 to get a value from 0 to 1 for compatibility with SwerveDrive as a RightX
    frontAngle = heading;
    stopTime = time;
    targetRotation = rotation;
    drivetrain = drive;

    addRequirements(drivetrain);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    distanceTraveled = 0.0;

    timer.start();
    startTime = timer.get();

    // Reset the encoder distances
    drivetrain.resetDistance();

    //ntables.zeroPiGyro();

    gyroOffset = 0.0;

    //The constants for these need to be figured out
    pidFrontAngle = new PIDController(kP_DriveAngle, kI_DriveAngle, kD_DriveAngle);


  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (firstRun) {
      
      drivetrain.resetDistance();

      gyroOffset = 0;
      SmartDashboard.putNumber("Gyro Offset", gyroOffset);

      firstRun = false;
    }

    // Calculate heading correction based on gyro reading and target heading
    //currentGyroAngle = drivetrain.getGyroAngle();
    //currentGyroAngle = drivetrain.getGyroAngle();
    currentGyroAngle = drivetrain.getGyroYaw();
  
    targetRotation = -pidFrontAngle.calculate(Math.toRadians(currentGyroAngle), Math.toRadians(frontAngle));
    

    SmartDashboard.putNumber("DriveSpeed", targetSpeed);
    // Enforce minimum speed
    // if (Math.abs(driveSpeed) < kAutoDriveSpeedMin) {
    //   angleCorrection = 0;
    //   if (driveSpeed < 0) {
    //     driveSpeed = -kAutoDriveSpeedMin;
    //   } else {
    //     driveSpeed = kAutoDriveSpeedMin;
    //   }
    // }

    double leftStickY = -targetSpeed * Math.cos(Math.toRadians(targetAngle));
    double leftStickX = targetSpeed * Math.sin(Math.toRadians(targetAngle));
    SmartDashboard.putNumber("Left Stick X", leftStickX);
    SmartDashboard.putNumber("Left Stick Y", leftStickY);


    // Run the drive
    if(isFieldOriented){
      drivetrain.driveFieldRelative(leftStickX, leftStickY, targetRotation);
    } else {
      drivetrain.driveRobotRelative(leftStickX, leftStickY, targetRotation);
    }
    

    // calculate driven distance
    distanceTraveled = drivetrain.calculateDriveDistance();
    SmartDashboard.putNumber("Distance Traveled", distanceTraveled);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.stopDrive();
    distanceTraveled = 0.0;

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean thereYet = false;
    double time = timer.get();

    // Get distance travelled
    distanceTraveled = drivetrain.calculateDriveDistance();

    // Check for stopping conditions
    if (distanceTraveled >= targetDriveDistance) {

       thereYet = true;

    } else if (time - startTime >= stopTime) {

      thereYet = true;

    } else if (killAuto == true) {

      thereYet = true;

    }

    return thereYet;
  }

}

