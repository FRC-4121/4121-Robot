// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.ArmExtend;
import edu.wpi.first.wpilibj.Timer;

public class AutoExtendArm extends CommandBase {
  
  private ArmExtend arm;
  private Timer timer;
  private double startTime;
  private double stopTime;
  private double targetPosition;
  private double targetLength;
  private double currentPosition;
  private double tolerance;
  private boolean forward; //this shows if the arm is extending forward or backward
  private int autoPosition; 
  
  
  /** Creates a new AutoExtendArm. */
  public AutoExtendArm(ArmExtend army, int position, double time) {
    
    arm = army;
    stopTime = time;
    autoPosition = position;
    
    addRequirements(arm);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // Start the timer and get initial time
    timer.start();
    startTime = timer.get();
  
    tolerance = 1000; //This needs to be found
    targetPosition = 0;

    //Set the target length based off of the expected position
    if (autoPosition == 1) {
      targetLength = ExtendStartPosition;
    }
    else if (autoPosition == 2) {
      targetLength = ExtendFloorPosition;
    }
    else if (autoPosition == 3) {
      targetLength = ExtendMidPosition;
    }
    else if (autoPosition == 4) {
      targetLength = ExtendHighPosition;
    }
    
    //Default is going up
    forward = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPosition = arm.getExtendEncoder();

    //Converting target length to encoder units
    targetPosition = extendSlope * targetLength + extendIntercept;

    if (targetPosition > currentPosition) {
      forward = true;
    } else{
      forward = false;
    }
    
    arm.extendArmToPosition(targetPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //Stop Arm
    arm.extendArm(0);
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
    if (currentPosition >= targetPosition-tolerance || currentPosition <= targetPosition + tolerance) {
      doneYet = true;
    }
    if (forward) {
      if (arm.getExtendSwitchValue()) {
        doneYet = true;
      }
    } else if (arm.getHomeSwitchValue()) {

      doneYet = true;
    }    
    
    return doneYet;
  }
}
