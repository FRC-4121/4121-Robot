// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.MechanismConstants.*;
import edu.wpi.first.wpilibj.Timer;

public class AutoShootCommand extends Command {

  private Shooter shooter;
  private Processor processor;
  private Timer timer = new Timer();
  private double startTime;
  private double endTime;
  private double delayTime;

  /** Creates a new AutoShootCommand. */
  public AutoShootCommand(Shooter shoot, Processor process, Double stopTime, Double delay) {

    shooter = shoot;
    processor = process;
    endTime = stopTime;
    delayTime = delay;
    
    addRequirements(shooter, processor);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    startTime = timer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = timer.get();
    double elapsedTime = currentTime - startTime;

    if((elapsedTime >= delayTime) && noteOnBoard)
    {
     shooter.runShooter(1.0);
     processor.runProcessor(1.0);  
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.runShooter(0);
    processor.runProcessor(0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     boolean doneYet = false;
    
    if (noteOnBoard == false)
    {
      doneYet = true;
    }
    if(timer.get() >= endTime-startTime)
    {
      doneYet = true;
    }
    if(killAuto == true )
    {
      doneYet = true;
    }

    return doneYet;
  }
  }
