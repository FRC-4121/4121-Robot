// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Processor;
import frc.robot.Constants.*;

public class RunProcessorBack extends Command {

  private Processor processor;

  /** Creates a new RunIntake. */
  public RunProcessorBack(Processor process) {

    processor = process;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    processor.runProcessor(-0.2);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    processor.runProcessor(0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
