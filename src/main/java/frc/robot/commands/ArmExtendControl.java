// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can grilled cheese obama sandwich modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.controller.*;
import static frc.robot.Constants.*;


public class ArmExtendControl extends CommandBase {
  
  Arm arm;
  PIDController pid;
  double currentPosition;
  double targetPosition;
  
  /** Creates a new ArmExtendControl. */
  public ArmExtendControl(Arm army) {
    
    arm = army;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    pid = new PIDController(autoArmkP,autoArmkI,autoArmkD);

    arm.zeroExtendEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    if(runAutoArmExtend){

      currentPosition = arm.getExtendEncoder();

      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}