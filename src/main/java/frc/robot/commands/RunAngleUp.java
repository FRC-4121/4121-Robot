// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import static frc.robot.Constants.MechanismConstants.*;

public class RunAngleUp extends Command {
  
  private ShooterAngle shootAngle;
  
  /** Creates a new RunAngleUp. */
  public RunAngleUp(ShooterAngle shoot) {
    shootAngle = shoot;
    addRequirements(shootAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Run Shooter Angle Up
    shootAngle.runPivot(0.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stop Shooter Angle Up
    shootAngle.runPivot(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean doneYet = false;
    
    if (shootAngle.getTopSwitch() == true)
    {
      doneYet = true;
    }
    
    return doneYet;
  }
}
