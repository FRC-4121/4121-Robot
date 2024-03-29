// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.NetworkBooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveWPI;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.*;


public class AutoPickupNote extends Command {

  private SwerveDriveWPI drive;
  private NetworkTableQuerier table;

  private double startTime;
  private double stopTime;
  private double notesFound;

  private int closestNote;
  private double closestDistance;
  private double noteDistance;
  private double noteOffset;
  private double noteOffsetTolerance;
  
  private double xSpeedSlope;
  private double xBaseSpeed;
  private double yBaseSpeed;
  private double rotBaseSpeed;
  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  private PIDController wpiPIDController;

  private Timer timer = new Timer();

  /** Creates a new AutoPickupNote. */
  public AutoPickupNote(SwerveDriveWPI swerve, NetworkTableQuerier ntable, double rotation, double time) {

    drive = swerve;
    table = ntable;
    stopTime = time;
    rotSpeed = rotation;
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Start timer and get current time
    timer.start();
    startTime = timer.get();

    // Initialize class variables
    closestNote = 0;
    closestDistance = 100000;//Really large because no rings will ever be that far
    noteOffsetTolerance = 1.0;
    xBaseSpeed = 0.15; //0.15
    yBaseSpeed = 0.4; //0.4
    rotBaseSpeed = 0.5;
    xSpeed = 0;
    ySpeed = 0;
    //rotSpeed = 0;
    noteOffset = -9999;
    xSpeedSlope = 0.02;

    // Create PID controller
    wpiPIDController = new PIDController(kAutoDrivePIDkp, kAutoDrivePIDkI, kAutoDrivePIDkD);
    wpiPIDController.setTolerance(1.0,5);

    // Set slow speed
    LinearSpeed = SlowMaxLinearSpeed;
    RotationalSpeed = SlowRadiansPerSecond;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Checking how many notes found
    notesFound = table.getRingsFound("CAM1");

    // Calculate drived action based on notes found
    if(notesFound > 0){

      // Determine the closest note
      if(notesFound > 1){
        for(int i = 0; i < notesFound; i++){
          if(table.getRingInfo("CAM1",i,"distance") < closestDistance){
            closestDistance = table.getRingInfo("Cam1",i,"distance");
            closestNote = i;
          }
        }
      }

      // Get closest note distance and offset
      noteDistance = table.getRingInfo("CAM1",closestNote,"distance");
      noteOffset = table.getRingInfo("CAM1",closestNote,"offset");

      // Determine drive direction based on note offset
      if (Math.abs(noteOffset) > noteOffsetTolerance) {

        // Get PID output
        double pidOutput = wpiPIDController.calculate(noteOffset,0);
        SmartDashboard.putNumber("PID Out", pidOutput);

        // Crab / rotate to note
        if (noteOffset > 0) {
          xSpeed = -xBaseSpeed * -pidOutput;  // SEth:  I put this structure in on Sunday.  Not sure if the x direction is correct
        } else {
          xSpeed = xBaseSpeed * -pidOutput;
        }        
        ySpeed = -yBaseSpeed;
        //rotSpeed = rotBaseSpeed * -pidOutput;

      } else {

        // Drive straight to note
        xSpeed = 0;
        ySpeed = -yBaseSpeed * 1.5;
        //rotSpeed = 0;
      }

    } else {

        // Drive straight to note
        xSpeed = 0;
        ySpeed = -yBaseSpeed * 1.5;
        //rotSpeed = 0;

    }
  


    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("NoteOffset", noteOffset);
    SmartDashboard.putNumber("Note Dist", noteDistance);

    // Run the drive
    if(isFieldOriented){
      drive.driveFieldRelative(xSpeed, ySpeed, rotSpeed);
    } else {
      drive.driveRobotRelative(xSpeed, ySpeed, rotSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Stop the drive motors
    drive.stopDrive();

    // Set fast drive speed
    LinearSpeed = MaxLinearSpeed;
    RotationalSpeed = MaxRadiansPerSecond;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean thereYet = false;

    double time = timer.get();

    if (noteOnBoard == true) {
      thereYet = true;
    }
    else if (stopTime <= time - startTime) {
      thereYet = true;
    }
    else if (killAuto == true)
    {
      thereYet = true;
    }

    return thereYet;
  }
}
