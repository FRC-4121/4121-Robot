// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class RunAngle extends Command {
  private ShooterAngle shootAngle;
  private Direction dir;
  
  private static final double ManualAngleMotorSpeed = 0.25;

  public static enum Direction {
    UP,
    DOWN,
  }

  /** Creates a new RunAngleUp. */
  public RunAngle(ShooterAngle shootAngle, Direction dir) {
    this.shootAngle = shootAngle;
    this.dir = dir;
    addRequirements(shootAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run Shooter Angle Up
    switch (dir) {
      case UP:
        if (!shootAngle.getTopSwitch()) {
          shootAngle.runPivot(ManualAngleMotorSpeed);
        }
        break;
      case DOWN:
        if (!shootAngle.getBottomSwitch()) {
          shootAngle.runPivot(-ManualAngleMotorSpeed);
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop Shooter Angle Up
    shootAngle.runPivot(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
