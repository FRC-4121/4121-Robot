// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.killAuto;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KillAutoCommand extends Command {
  /** Creates a new KillAutoCommand. */
  public KillAutoCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    killAuto = !killAuto;

    SmartDashboard.putBoolean("Kill Auto", killAuto);
    // if kill button clicked execute
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
