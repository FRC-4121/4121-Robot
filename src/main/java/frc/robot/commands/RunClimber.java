// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pneumatics.ClimberDirection;

public class RunClimber extends Command {
  private Pneumatics pneumatics;
  private ClimberDirection dir;

  public RunClimber(Pneumatics pneumatics, ClimberDirection dir) {
    this.pneumatics = pneumatics;
    this.dir = dir;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pneumatics.runClimber(dir);
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
