// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class DelayCommand extends Command {
  
  private Timer timer;
  private double startTime;
  private double stopTime;
  
  /** Creates a new DelayCommand. */
  public DelayCommand(double time) {
    
    stopTime = time;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Start the timer and get initial time
    timer = new Timer(); 
    timer.start();
    startTime = timer.get();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() - startTime >= stopTime)
    {
      return true;
    } else {
      return false;
    }
  }
}
