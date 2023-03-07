// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.ArmExtend;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoExtendArm extends CommandBase {
  
  private ArmExtend arm;
  private Timer timer;
  private double startTime;
  private double stopTime;
  private double targetPosition;
  private double currentPosition;
  private double tolerance;
  private boolean forward; //this shows if the arm is extending forward or backward 
  
  
  /** Creates a new AutoExtendArm. */
  public AutoExtendArm(ArmExtend army, double position, double time) {
    
    arm = army;
    stopTime = time;
    targetPosition = position;
    
    addRequirements(arm);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // Start the timer and get initial time
    timer = new Timer();
    timer.start();
    startTime = timer.get();
  
    tolerance = 350; //This needs to be found
    
    //Default is going up
    forward = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPosition = arm.getExtendEncoder();

    if(currentPosition < targetPosition){
      arm.extendArm(autoExtendSpeed);
      forward = true;
    } else{
      arm.extendArm(-autoExtendSpeed);
      forward = false;
    }

    SmartDashboard.putNumber("Extend Position", currentPosition);
    
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
    if (currentPosition >= targetPosition-tolerance && currentPosition <= targetPosition + tolerance) {
      doneYet = true;
    }
    // if (forward) {
    //  if (arm.getExtendSwitchValue()) {
    //     doneYet = true;
    //    }
    //  } else if (arm.getHomeSwitchValue()) {

    //    doneYet = true;
    // }    
    
    return doneYet;
  }
}
