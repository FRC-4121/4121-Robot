// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;

public class TakeInNote extends TimeoutCommand {

  // Declare local variables
  private Intake noteIntake;
  private Processor noteProcessor;

  /** Creates a new TakeInNote. */
  public TakeInNote(Intake intake, Processor process, double time) {
    super(time);
    // Set local variables
    noteIntake = intake;
    noteProcessor = process;

    addRequirements(noteIntake, noteProcessor);
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
    return Constants.noteOnBoard || super.isFinished();
  }
}
