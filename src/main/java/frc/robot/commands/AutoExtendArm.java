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
  
  
  /** Creates a new AutoExtendArm. */
  public AutoExtendArm(ArmExtend army, double time, double length) {
    
    arm = army;
    stopTime = time;
    targetPosition = length;
    targetLength = length;
    
    addRequirements(arm);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // Start the timer and get initial time
    timer.start();
    startTime = timer.get();
  
    tolerance = 1000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPosition = arm.getExtendEncoder();

    //Converting target length to encoder units
    targetPosition = extendSlope * targetLength + extendIntercept;
    
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
   
    if(killAuto == true)
    {
      doneYet = true;
    } 
    if(timer.get() >= stopTime)
    {
      doneYet = true;
    }
    if(currentPosition >= targetPosition-tolerance || currentPosition <= targetPosition + tolerance){
      doneYet = true;
    }
    if(arm.getExtendSwitchValue()){
      doneYet = true;
    }
    
    
    return doneYet;
  }
}
