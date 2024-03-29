// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Processor;
import frc.robot.Constants.*;

public class RunIntake extends Command {

  private Intake noteIntake;
  private Processor processor;

  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, Processor process) {

    noteIntake = intake;
    processor = process;

    addRequirements(intake);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    noteIntake.runIntake(0.5);
    processor.runProcessor(0.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    noteIntake.stopIntake();
    processor.runProcessor(0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
