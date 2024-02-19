// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

public class TakeInNote extends Command {

  // Declare local variables
  private Intake noteIntake;
  private Processor noteProcessor;
  private double startTime;
  private double stopTime;
  private Timer timer;

  /** Creates a new TakeInNote. */
  public TakeInNote(Intake intake, Processor process, double time) {

    // Set local variables
    noteIntake = intake;
    noteProcessor = processor;
    stopTime = time;

    addRequirements(noteIntake, noteProcessor);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Start timer and get current time
    timer = new Timer();
    timer.start();
    startTime = timer.get();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Run intake and processor motors
    noteProcessor.runProcessor(0.5);
    noteIntake.runIntake(0.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Stop motors
    noteIntake.stopIntake();
    noteProcessor.stopProcessor();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Initialize return flag
    boolean thereYet = false;

    // Get the current time
    double time = timer.get();

    // Check for stop conditions
    if (noteOnBoard) {
      thereYet = true;
    }
    else if (stopTime <= time - startTime) {
      thereYet = true;
    }

    // Return flag
    return thereYet;

  }
  
}
