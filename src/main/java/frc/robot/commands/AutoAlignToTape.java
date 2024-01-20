// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.*;

public class AutoAlignToTape extends Command {

  // Declare class variables
  private final SwerveDrive drivetrain;
  private NetworkTableQuerier ntables;
  private double targetSpeed;
  private double stopTime;
  private double startTime;
  private double tapeOffset;
  private double leftStickX;
  private double leftStickY;
  private double gyroOffset;
  private double currentGyroAngle;
  private double targetRotation;
  private double frontAngle;
  private double chassisRotation;
  private boolean firstRun;
  private double kP;

  private Timer timer;
  private PIDController pidFrontAngle;


  /** Creates a new AutoAlignToTape. */
  public AutoAlignToTape(SwerveDrive drive, double speed, double time, NetworkTableQuerier table) {

      targetSpeed = speed;
      stopTime = time;
      ntables = table;
      drivetrain = drive;
  
      addRequirements(drivetrain);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    
    // Initialize tape offset
    tapeOffset = 0.0;

    // Initialize drive variables
    leftStickX = 0.0;
    leftStickY = 0.0;
    gyroOffset = 0.0;
    currentGyroAngle = 0.0;
    targetRotation = 0.0;
    frontAngle = 0.0;
    chassisRotation = 0.0;

    kP = 0.38;

    firstRun = true;

    pidFrontAngle = new PIDController(kP_DriveAngle, kI_DriveAngle, kD_DriveAngle);

    // Start the timer and get the current time
    timer = new Timer();
    timer.start();
    startTime = timer.get();

    SmartDashboard.putBoolean("Align Cone End", false);
    SmartDashboard.putBoolean("Align Tapes End", false);
    SmartDashboard.putBoolean("Align Tol End", false);
    SmartDashboard.putBoolean("Align Time End", false);

  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (firstRun) {
      
      gyroOffset = ntables.getPiGyro();
      SmartDashboard.putNumber("Gyro Offset", gyroOffset);

      firstRun = false;

    }

    // Calculate angle correction based on gyro reading
    currentGyroAngle = ntables.getPiGyro() - gyroOffset;
    SmartDashboard.putNumber("Current Gyro", currentGyroAngle);
    
    targetRotation = pidFrontAngle.calculate(currentGyroAngle / 360.0, frontAngle) + chassisRotation;
    // if (targetRotation > 1) {
    //   targetRotation = 1;
    // } else if (targetRotation < -1) {
    //   targetRotation = -1;
    // }

    // If we see the vision tape, move robot
    if (ntables.getVisionDouble("TapesFound") > 0) {


      // Get correct offset based on which tape was found
      double targetTapeOffset = -7.5;
      double tapeY = ntables.getVisionDouble("Tapes.0.y");
      if (tapeY > 280) {
        targetTapeOffset = targetTapeOffsetLow;
      } else {
        targetTapeOffset = targetTapeOffsetHigh;
      }

      // Get the offset of the closest vision tape
      tapeOffset = ntables.getVisionDouble("Tapes.0.offset") - targetTapeOffset;
      SmartDashboard.putNumber("Tape Delta", tapeOffset);

      double alignFactor = Math.abs(tapeOffset) * kP;


      // Determine which way the robot needs to move
      if (tapeOffset < -visionTapeTolerance) {

        // Robot is left of target so need to move right
        leftStickX = targetSpeed * alignFactor;

      } else if (tapeOffset > visionTapeTolerance) {

        // Robot is right of target so need to move left
        leftStickX = -targetSpeed * alignFactor;

      } else {

        // Robot is on target
        leftStickX = 0.0;

      }

      // Run the drive
      if(getCone){
      drivetrain.drive(leftStickX, leftStickY, targetRotation);
      }

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Stop the drive
    drivetrain.stopDrive();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Initialize stop check flag
    boolean thereYet = false;

    // Get the current time
    double time = timer.get();

    // Get correct offset based on which tape was found
    double targetTapeOffset = -7.5;
    double tapeY = ntables.getVisionDouble("Tapes.0.y");
    if (tapeY > 280) {
      targetTapeOffset = targetTapeOffsetLow;
    } else {
      targetTapeOffset = targetTapeOffsetHigh;
    }
    
    // Get tape offset
    tapeOffset = ntables.getVisionDouble("Tapes.0.offset") - targetTapeOffset;
    SmartDashboard.putNumber("Tape Delta", tapeOffset);

    // Check stopping conditions
    if(!getCone) {

      thereYet = true;
      SmartDashboard.putBoolean("Align Cone End", true);

    } else if (ntables.getVisionDouble("TapesFound") == 0) {

      thereYet = true;
      SmartDashboard.putBoolean("Align Tapes End", true);

    } else if (tapeOffset >= -visionTapeTolerance && tapeOffset <= visionTapeTolerance) {

      thereYet = true;
      SmartDashboard.putBoolean("Align Tol End", true);

    } else if (time - startTime >= stopTime) {

      thereYet = true;
      SmartDashboard.putBoolean("Align Time End", true);

    }

    return thereYet;

  }
}
