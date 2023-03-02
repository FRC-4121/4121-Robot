// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.Pneumatics;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;

public class AutoArmRotation extends CommandBase {
  
  private ArmRotate arm;
  private Pneumatics pneumatic;
  private boolean isReleased;
  private double targetAngle;
  private double targetPosition;
  private double currentPosition;
  private Timer timer;
  private double startTime;
  private double stopTime;
  private double tolerance;
  private int autoPosition;
  
  /** Creates a new AutoArmRotation. */
  public AutoArmRotation(ArmRotate armRotate, int position, double time, Pneumatics pneumatics) {
    
    arm = armRotate;
    autoPosition = position;
    stopTime = time;
    pneumatic = pneumatics;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     
    // Start the timer and get initial time
     timer.start();
     startTime = timer.get();

     isReleased = false;

     tolerance = 1000;

     //Set the target Angle based off of the expected position
    if (autoPosition == 1) {
      targetAngle = RotateStartPosition;
    }
    else if (autoPosition == 2) {
      targetAngle = RotateFloorPosition;
    }
    else if (autoPosition == 3) {
      targetAngle = RotateMidPosition;
    }
    else if (autoPosition == 4) {
      targetAngle = RotateHighPosition;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //On the first run, release the brake so we can move
    if (!isReleased) {
      pneumatic.releaseBrake();
    }

    //Calculating the target position based off of an equation found by testing
    targetPosition = rotateSlope * targetAngle + rotateIntercept;
    
    //Get the currentPosition
    currentPosition = arm.getMasterEncoder();
    
    //Call method to rotate to an encoder postion with smoothing
    arm.rotateToPosition(targetPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //Apply the brake so we stop moving
    pneumatic.applyBrake();
    isReleased = false;

    //Stop rotating
    arm.rotate(0);

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
    if (currentPosition >= targetPosition-tolerance || currentPosition <= targetPosition + tolerance)
    {
      doneYet = true;
    }
    
    return doneYet;
  }
}
