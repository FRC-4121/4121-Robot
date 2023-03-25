// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.ExtraClasses.NetworkTableQuerier;

public class AutoAlignToTape extends CommandBase {

  // Declare class variables
  private final SwerveDrive drivetrain;
  private NetworkTableQuerier ntables;
  private double targetSpeed;
  private double stopTime;
  private double startTime;
  private double tapeOffset;
  private double leftStickX;
  private double leftStickY;
  private double rightStickX;
  private Timer timer;

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
    rightStickX = 0.0;

    // Start the timer and get the current time
    timer.start();
    startTime = timer.get();

  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // If we see the vision tape, move robot
    if (ntables.getVisionDouble("TapesFound") > 0) {

      // Get the offset of the closest vision tape
      tapeOffset = ntables.getVisionDouble("Tapes.0.offset") - targetTapeOffset;

      // Determine which way the robot needs to move
      if (tapeOffset < -visionTolerance) {

        // Robot is left of target so need to move right
        rightStickX = targetSpeed;

      } else if (tapeOffset > visionTolerance) {

        // Robot is right of target so need to move left
        rightStickX = -targetSpeed;

      } else {

        // Robot is on target
        rightStickX = 0.0;

      }

      // Run the drive
      drivetrain.drive(leftStickX, leftStickY, rightStickX);

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

    // Get tape offset
    tapeOffset = ntables.getVisionDouble("Tapes.0.offset") - targetTapeOffset;

    // Check stopping conditions
    if (tapeOffset >= -visionTolerance && tapeOffset <= visionTolerance) {

      thereYet = true;

    } else if (time - startTime >= stopTime) {

      thereYet = true;

    }

    return thereYet;

  }
}
