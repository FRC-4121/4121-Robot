// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ExtraClasses.PIDControl;
import edu.wpi.first.math.controller.*;

public class AutoBalance extends CommandBase {

  /** Creates a new AutoBalance. */
  private final SwerveDrive drivetrain;
  private double targetAngle;
  private double targetSpeed;
  private double targetRotation;
  private double stopTime;
  private double startTime;
  private double climbStartTime;
  private double climbTimeLimit;
  private Boolean isClimbing;
  private Boolean reachedTop;
  private Boolean isBalanced;
  private double currentPitch;
  private double startingPitch;
  private double climbThreshold; //Threshhold for seeing if we have started climbing
  private double pitchTolerance; //Tolerance for seeing if we have traveled too far
  private double topThreshold; //Threshold for determining if we have reached the top
  private double speedMultiplier; 
  private double balanceCounter; //Count how long we have been balanced
  private double balanceTargetCount;
  private double topCounter;
  private double topTargetCount;
  private double chassisRotation;
  private double frontAngle;
  private Boolean firstRun = true;
  private double gyroOffset;
  private double currentGyroAngle = 0;
  private double balanceStartTime;
  private double balanceTimeLimit;

  private NetworkTableQuerier ntables;

  private PIDController pidFrontAngle;

  private Timer timer = new Timer();

  //distance is in inches

  public AutoBalance(SwerveDrive drive, double speed, double ang, double rot, double heading, double time, NetworkTableQuerier table ) {
   
    ntables = table;
    targetSpeed = speed;
    targetAngle = ang;
    stopTime = time;
    drivetrain = drive;
    chassisRotation = rot;
    frontAngle = heading / 360;

    addRequirements(drivetrain);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    isClimbing = false;
    reachedTop = false;
    isBalanced = false;

    climbThreshold = 7;
    topThreshold = 1.0;
    pitchTolerance = 0.5;
    speedMultiplier = 1;
    balanceCounter = 0;
    balanceTargetCount = 20;
    topCounter = 0.0;
    topTargetCount = 17;
    startingPitch = ntables.getNavXDouble("Orientation.1"); 
    gyroOffset = 0.0;
    climbTimeLimit = 1.0;
    balanceTimeLimit = 2.0;

    timer.start();
    startTime = timer.get();

    pidFrontAngle = new PIDController(kP_DriveAngle, kI_DriveAngle, kD_DriveAngle);

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(firstRun) {

      gyroOffset = ntables.getPiGyro();
      SmartDashboard.putNumber("Gyro Offset", gyroOffset);

      balanceCounter = 0.0;
      topCounter = 0.0;
      isClimbing = false;
      reachedTop = false;
      isBalanced = false;

      firstRun = false;
    }

    // Calculate angle correction based on gyro reading
    currentGyroAngle = ntables.getPiGyro() - gyroOffset;
    SmartDashboard.putNumber("Current Gyro", currentGyroAngle);
        
    targetRotation = pidFrontAngle.calculate(currentGyroAngle / 360.0, frontAngle) + chassisRotation;

    //Getting pitch reading from pi
    currentPitch = ntables.getNavXDouble("Orientation.1") - startingPitch;

    //Checking if we are currently climbing based on pitch reading from pi
    if (!isClimbing) {
      if (Math.abs(currentPitch) >= climbThreshold) {
        isClimbing = true;
        speedMultiplier = 0.75;
        climbStartTime = timer.get();
      }
    }

    //Checking if we have reached the top yet
    if(isClimbing && !reachedTop) {
      if (Math.abs(currentPitch) < topThreshold) {
        if (topCounter >= topTargetCount) {
          isClimbing = false;
          reachedTop = true;
          balanceStartTime = timer.get();
          speedMultiplier = 0.0;
        }
        else if ((timer.get() - climbStartTime) > climbTimeLimit) {
          isClimbing = false;
          reachedTop = true;
          balanceStartTime = timer.get();
          speedMultiplier = 0.0;
        } else {
          topCounter++;
        }
      }
    }

    //Seeing if we have to make corrections after we have reached the top
    if (reachedTop) {
      targetRotation = 0;
      speedMultiplier = 0.0;
      if ((timer.get() - balanceStartTime) > balanceTimeLimit) {
        if (Math.abs(currentPitch) <= pitchTolerance) {
          if (balanceCounter >= balanceTargetCount) {
            isBalanced = true;
          } else {
            isBalanced = true;
            balanceCounter++;
          }
        } else if (currentPitch > 0) {
          speedMultiplier = -0.4;
          balanceCounter = 0;
        } else {
          speedMultiplier = 0.4;
          balanceCounter = 0;
        }
      }
    }

    SmartDashboard.putNumber("Speed Multiplier", speedMultiplier);
    SmartDashboard.putNumber("Balance Counter", balanceCounter);

    SmartDashboard.putBoolean("Is Climbing",isClimbing);
    SmartDashboard.putBoolean("Reached Top",reachedTop);
    SmartDashboard.putBoolean("Is Balanced",isBalanced);

    SmartDashboard.putNumber("DriveSpeed", targetSpeed);
    SmartDashboard.putNumber("Current Pitch", currentPitch);

    double leftStickY =  -targetSpeed * Math.cos(Math.toRadians(targetAngle)) * speedMultiplier;
    double leftStickX =  targetSpeed * Math.sin(Math.toRadians(targetAngle)) * speedMultiplier;
    SmartDashboard.putNumber("Left Stick X", leftStickX);
    SmartDashboard.putNumber("Left Stick Y", leftStickY);


    // Run the drive
    drivetrain.drive(leftStickX, leftStickY, targetRotation);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //Stop driving when we are done
    drivetrain.stopDrive();

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean thereYet = false;
    double time = timer.get();

    if (isBalanced) {
       thereYet = true;
    } else if (time - startTime >= stopTime) {
      thereYet = true;
    } else if (killAuto == true) {
      thereYet = true;
    }

    return thereYet;
  }

}
