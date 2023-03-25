// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class RunWristDown extends CommandBase {
 
  private Wrist m_wrist;
  private Timer timer;
  private double startMoveTime;
  private double startingWristPosition;
 
  /** Creates a new RunWristForward. */
  public RunWristDown(Wrist wrist) {
    
    m_wrist = wrist;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer = new Timer();
    timer.start();
    startMoveTime = timer.get();

    startingWristPosition = currentWristPosition;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_wrist.move(-wristSpeed);
    
    if (GrabbedCone) {

      // Add because we are moving it down
      currentWristPosition = startingWristPosition + (wristSlopeDownCone * (timer.get() - startMoveTime) + wristIntercept);

    } else {

      // Add because we are moving it down
      currentWristPosition = startingWristPosition + (wristSlopeDownEmpty * (timer.get() - startMoveTime) + wristIntercept);

    }
    
    // SmartDashboard.putNumber("Wrist Position", m_wrist.getEncoderPos());
    SmartDashboard.putNumber("WristPosition",currentWristPosition);
    
    SmartDashboard.putNumber("Wrist Time", timer.get()-startMoveTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_wrist.move(0);
    
    if (GrabbedCone) {

      // Add because we are moving it down
      currentWristPosition = startingWristPosition + (wristSlopeDownCone * (timer.get() - startMoveTime) + wristIntercept);

    } else {

      // Add because we are moving it down
      currentWristPosition = startingWristPosition + (wristSlopeDownEmpty * (timer.get() - startMoveTime) + wristIntercept);

    }

    // SmartDashboard.putNumber("Wrist Position", m_wrist.getEncoderPos());
    SmartDashboard.putNumber("WristPosition",currentWristPosition);
    SmartDashboard.putNumber("Wrist Time", timer.get()-startMoveTime);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
