// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.ArmRotate;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;

public class AutoArmRotation extends CommandBase {
  
  private ArmRotate arm;
  private double targetAngle;
  private double targetPosition;
  private double currentPosition;
  private Timer timer;
  private double startTime;
  private double stopTime;
  
  /** Creates a new AutoArmRotation. */
  public AutoArmRotation(ArmRotate armRotate, double angle, double time) {
    
    arm = armRotate;
    targetAngle = angle;
    stopTime = time;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     
    // Start the timer and get initial time
     timer.start();
     startTime = timer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetPosition = rotateSlope * targetAngle + rotateIntercept;
    currentPosition = arm.getMasterEncoder();
    
    arm.rotateToAngle(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

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
    if(currentPosition == targetPosition)
    {
      doneYet = true;
    }
    
    return doneYet;
  }
}
