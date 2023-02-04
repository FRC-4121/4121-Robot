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

public class AutoBalance extends CommandBase {

  /** Creates a new AutoBalance. */
  private final SwerveDrive drivetrain;
  private double targetAngle;
  private double targetSpeed;
  private double stopTime;
  private double startTime;
  private Boolean isClimbing;
  private Boolean reachedTop;
  private Boolean isBalanced;
  private double currentPitch;
  private double pitchThreshold; //Threshhold for seeing if we have started climbing
  private double pitchTolerance; //Tolerance for seeing if we have traveled too far
  private double speedMultiplier; 
  private double balanceCounter; //Count how long we have been balanced
  private double balanceTargetCount;

  private NetworkTableQuerier ntables;

  private Timer timer = new Timer();

  //distance is in inches

  public AutoBalance(SwerveDrive drive, double speed, double ang, double time, NetworkTableQuerier table ) {
   
    ntables = table;
    targetSpeed = speed;
    targetAngle = ang;
    stopTime = time;
    drivetrain = drive;

    addRequirements(drivetrain);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    isClimbing = false;
    reachedTop = false;
    isBalanced = false;

    pitchThreshold = -7;
    pitchTolerance = 1;
    speedMultiplier = 1;
    balanceCounter = 0;
    balanceTargetCount = 20;

    timer.start();
    startTime = timer.get();

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Getting pitch reading from pi
    currentPitch = ntables.getNavXDouble("Orientation.1");

    //Checking if we are currently climbing based on pitch reading from pi
    if (!isClimbing) {
      if (currentPitch <= pitchThreshold) {
        isClimbing = true;
        speedMultiplier = 0.5;
      }
    }

    //Checking if we have reached the top yet
    if(isClimbing && !reachedTop) {
      if (currentPitch >= pitchThreshold) {
        isClimbing = false;
        reachedTop = true;
        speedMultiplier = 0.5;
      }
    }

    //Seeing if we have to make corrections after we have reached the top
    if (reachedTop) {
      speedMultiplier = 0.0;
      if (Math.abs(currentPitch) <= pitchTolerance) {
        if (balanceCounter >= balanceTargetCount) {
          isBalanced = true;
        } else {
          balanceCounter++;
        }
      } else if (currentPitch > 0) {
        speedMultiplier = -0.5;
        balanceCounter = 0;
      } else {
        speedMultiplier = 0.5;
        balanceCounter = 0;
      }
    }

    SmartDashboard.putNumber("Speed Multiplier", speedMultiplier);
    SmartDashboard.putNumber("Balance Counter", balanceCounter);

    SmartDashboard.putBoolean("Is Climbing",isClimbing);
    SmartDashboard.putBoolean("Reached Top",reachedTop);
    SmartDashboard.putBoolean("Is Balanced",isBalanced);

    SmartDashboard.putNumber("DriveSpeed", targetSpeed);
    SmartDashboard.putNumber("Current Pitch", currentPitch);

    double leftStickY =  -targetSpeed * Math.cos(Math.toRadians(targetAngle) * speedMultiplier);
    double leftStickX =  targetSpeed * Math.sin(Math.toRadians(targetAngle) * speedMultiplier);
    SmartDashboard.putNumber("Left Stick X", leftStickX);
    SmartDashboard.putNumber("Left Stick Y", leftStickY);


    // Run the drive
    drivetrain.drive(leftStickX, leftStickY, 0);

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
