// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngle;
import static frc.robot.Constants.MechanismConstants.*;

public class AutoShooterAmpPos extends Command {
  
  private ShooterAngle shooter;
  
  /** Creates a new AutoShooterAmpPos. */
  public AutoShooterAmpPos(ShooterAngle shoot) {
    
    shooter = shoot;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  //if(!((shooter.getTopSwitch() == true) || shooter.getPosition() == AmpAngle))
  //{

  //Angle the shooter to the amp position
  //shooter.autoRunPivot(AmpAngle);

  //} 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  shooter.runPivot(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
