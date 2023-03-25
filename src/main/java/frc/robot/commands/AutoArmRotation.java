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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.*;

public class AutoArmRotation extends CommandBase {
  
  private ArmRotate arm;
  private Pneumatics pneumatic;
  private boolean isReleased;
  private double targetAngle;
  private double currentPosition;
  private Timer timer;
  private double startTime;
  private double stopTime;
  private double tolerance;
  private PIDController wpiPIDController;

  /** Creates a new AutoArmRotation. */
  public AutoArmRotation(ArmRotate armRotate, double angle, double time, Pneumatics pneumatics) {
    
    arm = armRotate;
    targetAngle = angle;
    stopTime = time;
    pneumatic = pneumatics;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     
    // Start the timer and get initial time
    timer = new Timer(); 
    timer.start();
    startTime = timer.get();

    isReleased = false;

    tolerance = 200;

    // Initialize PID controller
    wpiPIDController = new PIDController(autoArmkP, autoArmkI, autoArmkD);
    wpiPIDController.setTolerance(tolerance,500);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //On the first run, release the brake so we can move
    if (!isReleased) {
      pneumatic.releaseBrake();
      isReleased = true;
    }

    //Calculating the target position based off of an equation found by testing
    //targetPosition = rotateSlope * targetAngle + rotateIntercept;
    
    //Get the currentPosition
    currentPosition = arm.getMasterEncoder();

    double output = wpiPIDController.calculate(currentPosition,targetAngle);

    if (output > autoRotateSpeed) {
      output = autoRotateSpeed;
    }
    else if (output < -autoRotateSpeed) {
      output = -autoRotateSpeed;
    }

    arm.rotate(output);

    // if(currentPosition < targetAngle){
    //   arm.rotate(autoRotateSpeed);
    // } else{
    //   arm.rotate(-autoRotateSpeed);
    // }
    
    //Call method to rotate to an encoder postion with smoothing
    //arm.rotateToPosition(targetAngle);

    //Put values on Smart Dashboard
    SmartDashboard.putNumber("Rotate Position", arm.getMasterEncoder());  
    SmartDashboard.putNumber("Slave Encoder", arm.getSlaveEncoder());

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
    if (currentPosition >= targetAngle-tolerance && currentPosition <= targetAngle + tolerance)
    {
      doneYet = true;
    }

    SmartDashboard.putBoolean("Arm Rotate Done",doneYet);
    
    return doneYet;
  }
}
