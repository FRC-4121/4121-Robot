// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoMoveWrist extends CommandBase {
  
  private Wrist wrist;
  private Timer timer;
  private double startMoveTime;
  private double startTime;
  private double stopTime;
  private double targetPosition;
  private double tolerance;
  private double startingWristPosition;

  
  /** Creates a new AutoMoveWrist. */
  public AutoMoveWrist(Wrist wristy, double position, double time) {
    
    wrist = wristy;
    targetPosition = position;
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
    startMoveTime = timer.get();

    tolerance = 0.01;

    startingWristPosition = currentWristPosition;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //We have to decide if we want to rotate up or down
    if(currentWristPosition < targetPosition){
      
      wrist.move(-wristSpeed);

      //Add because we are moving it down
      currentWristPosition = startingWristPosition + (wristSlope * (timer.get()-startMoveTime) + wristIntercept);

      SmartDashboard.putNumber("WristPosition",currentWristPosition);

      SmartDashboard.putNumber("Wrist Time", timer.get()-startMoveTime);

    } else if (currentWristPosition > targetPosition){
      
      wrist.move(wristSpeed);

      //Subtract because we are moving the wrist back up
      currentWristPosition = startingWristPosition - (wristSlope * (timer.get()-startMoveTime) + wristIntercept);

      SmartDashboard.putNumber("WristPosition",currentWristPosition);

      SmartDashboard.putNumber("Wrist Time", timer.get()-startMoveTime);

    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    wrist.move(0);

    SmartDashboard.putNumber("WristPosition",currentWristPosition);

    SmartDashboard.putNumber("Wrist Time", timer.get()-startMoveTime);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean doneYet = false;
   
    if (killAuto == true)
    {
      doneYet = true;
    } 
    if (timer.get() - startTime >= stopTime)
    {
      doneYet = true;
    }
    if (currentWristPosition >= targetPosition - tolerance && currentWristPosition <= targetPosition + tolerance)
    {
      doneYet = true;
    }

    SmartDashboard.putBoolean("Arm Rotate Done",doneYet);
    
    return doneYet;
  }
}
