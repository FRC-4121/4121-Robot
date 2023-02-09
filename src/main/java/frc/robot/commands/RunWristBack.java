// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class RunWristBack extends CommandBase {
 
  private Wrist m_wrist;
 
  /** Creates a new RunWristForward. */
  public RunWristBack(Wrist wrist) {
    
    m_wrist = wrist;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_wrist.move(-0.2); // will need testing to confirm if value needs to be positive or negative
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_wrist.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}