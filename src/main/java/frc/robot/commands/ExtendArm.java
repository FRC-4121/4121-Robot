// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtend;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ExtendArm extends CommandBase {
  
  private ArmExtend m_arm;
  
  /** Creates a new RunArmForward. */
  public ExtendArm(ArmExtend arm) {
    
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_arm.extendArm(0.5);

    SmartDashboard.putNumber("Extend Position", m_arm.getExtendEncoder());
    SmartDashboard.putBoolean("Arm Limit Switch", m_arm.getHomeSwitchValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_arm.stopArm();

    //When we stop, we want it to hold its position, so the target is the latest value
    armTargetEncoder = m_arm.getExtendEncoder();
    
    SmartDashboard.putNumber("Extend Position", m_arm.getExtendEncoder());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    // if (m_arm.getExtendSwitchValue()) {
    //   return true;
    // } else {
      return false;
   // }
    
  }
}
