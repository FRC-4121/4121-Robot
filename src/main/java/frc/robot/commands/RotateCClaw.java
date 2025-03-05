// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import static frc.robot.Constants.MechanismConstants.*;

public class RotateCClaw extends Command {

  private CClaw myCClaw;
  private XboxController myXbox;

  private final SlewRateLimiter ySpeedLimiter;
  private double ySpeed;

  /** Creates a new RotateCClaw. */
  public RotateCClaw(CClaw claw, XboxController xbox) {

    myCClaw = claw;
    myXbox = xbox;

    ySpeedLimiter = new SlewRateLimiter(2);
    
    addRequirements(myCClaw);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ySpeed = -ySpeedLimiter.calculate(MathUtil.applyDeadband(myXbox.getLeftY(), 0.01))
        * RotateSpeed;

    myCClaw.rotate(ySpeed);
    
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
