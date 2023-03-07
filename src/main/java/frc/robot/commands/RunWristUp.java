// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class RunWristUp extends CommandBase {
 
  private Wrist m_wrist;
  private Timer timer;
  private double startMoveTime;
 
  /** Creates a new RunWristForward. */
  public RunWristUp(Wrist wrist) {
    
    m_wrist = wrist;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer = new Timer();
    timer.start();
    startMoveTime = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_wrist.move(wristSpeed);
    
    //Get the time that the wrist started moving
    startMoveTime = timer.get();

    //Subtract because we are moving the wrist back up
    currentWristPosition = currentWristPosition - (wristSlope * (timer.get()-startMoveTime) + wristIntercept);

    SmartDashboard.putNumber("WristPosition",currentWristPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_wrist.move(0);

    startMoveTime = timer.get();

    //Subtract because we are moving the wrist back up
    currentWristPosition = currentWristPosition - (wristSlope * (timer.get()-startMoveTime) + wristIntercept);

    SmartDashboard.putNumber("WristPosition",currentWristPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
