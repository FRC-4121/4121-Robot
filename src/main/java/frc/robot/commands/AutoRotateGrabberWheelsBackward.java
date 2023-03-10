// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
import edu.wpi.first.wpilibj.Timer;

public class AutoRotateGrabberWheelsBackward extends CommandBase {
  
  double startTime;
  double stopTime;
  Timer timer;
  Grabber grabber;

  /** Creates a new AutoRotateGrabberWheelsBackward. */
  public AutoRotateGrabberWheelsBackward(Grabber grab, Double time) {
    
    grabber = grab;
    stopTime = time;
    
    // Use addRequirements() here to declare subsystem dependencies.
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

    grabber.runIntake(-0.8);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    grabber.stopIntake();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   
   boolean doneYet = false;
   double time = timer.get();

    if (time - startTime >= stopTime) {
      doneYet = true;
    }
    
    return doneYet;
  }
}
