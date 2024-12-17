// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;

public class AutoShootCommand extends AutoCommand {
  private Shooter shooter;
  private Processor processor;
  private double delayTime;

  /** Creates a new AutoShootCommand. */
  public AutoShootCommand(Shooter shoot, Processor process, double time, double delay) {
    super(time);
    shooter = shoot;
    processor = process;
    delayTime = delay;

    addRequirements(shooter, processor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.noteOnBoard && currentTime() > delayTime && !super.isFinished()) {
      shooter.runShooterSpeaker(1.0);
      processor.runProcessor(1.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.runShooterSpeaker(0);
    processor.runProcessor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished() || !Constants.noteOnBoard;
  }
}
