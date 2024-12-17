// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pneumatics;

public class ToggleClimber extends Command {
  Pneumatics pneumatics;

  /** Creates a new ExtendClimber. */
  public ToggleClimber(Pneumatics pneumatics) {
    this.pneumatics = pneumatics;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If the climber is extended, retract the climber, else extent the climber
    if (pneumatics.isExtended()) {
      SmartDashboard.putBoolean("extended", false);
      pneumatics.retractClimber();
    } else {
      SmartDashboard.putBoolean("extended", true);
      pneumatics.extendClimber();
    }

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