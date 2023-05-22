// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotateArmUp extends CommandBase {
  
  ArmRotate arm;
  Pneumatics pneumatic;
  double currentSpeed;
  boolean isReleased;
  
  /** Creates a new RotateArm. */
  public RotateArmUp(ArmRotate army, Pneumatics pneumatics) {
    
    arm = army;
    pneumatic = pneumatics;
    
    addRequirements(arm,pneumatic);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    currentSpeed = 0;
    isReleased = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //On the first run, release the brake so we can move
    if (!isReleased) {
      pneumatic.releaseBrake();
    }

    // Increase the speed gradually to avoid bouncing
    if (currentSpeed < teleopRotateSpeed) {
      currentSpeed = currentSpeed + rotateRampRate;
    }
    arm.rotate(currentSpeed);

    //Put values on Smart Dashboard
    SmartDashboard.putNumber("Rotate Master", arm.getMasterEncoder());
    SmartDashboard.putNumber("Rotate Slave", arm.getSlaveEncoder());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Decrease the speed gradually to avoid bouncing
    while (currentSpeed > 0) {
      currentSpeed = currentSpeed - rotateRampRate;
      arm.rotate(currentSpeed);
    }
    

    //Apply the brake so we stop moving
    pneumatic.applyBrake();
    isReleased = false;

    //Put values on Smart Dashboard
    SmartDashboard.putNumber("Rotate Master", arm.getMasterEncoder());
    SmartDashboard.putNumber("Rotate Slave", arm.getSlaveEncoder());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
